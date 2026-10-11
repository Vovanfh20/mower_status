#!/usr/bin/env python3
"""Нода определения статуса робота-газонокосилки и фильтрации GPS.

Подписывается:
    /imu                                 sensor_msgs/Imu            (нужна только ориентация)
    /RoboMowerRealistic/gps              geometry_msgs/PointStamped (позиция, метры)
    /RoboMowerRealistic/gps/speed        std_msgs/Float32           (модуль скорости, м/с)

Публикует:
    /robot/status          std_msgs/String        ROTATED | MOVEMENT_ON_PLACE | ROBOT DRIVING
    /robot/gps             geometry_msgs/Vector3  сырая позиция, переупакованная в Vector3
    /robot/gps_filtered    geometry_msgs/Vector3  позиция после фильтра Калмана

Логика статуса (приоритет сверху вниз):
    1. Угол наклона от вертикали выше порога дольше выдержки  -> ROTATED
    2. Средняя скорость ниже порога дольше выдержки           -> MOVEMENT_ON_PLACE
    3. Иначе                                                  -> ROBOT DRIVING

Все пороги вынесены в параметры, значения по умолчанию получены измерением
на трёх логах — обоснование в README.
"""

import csv
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from mower_status.kalman import Vector3KF

STATUS_ROTATED = 'ROTATED'
STATUS_STUCK = 'MOVEMENT_ON_PLACE'
STATUS_DRIVING = 'ROBOT DRIVING'


def tilt_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Угол между собственной осью Z робота и вертикалью мира, в градусах.

    Третий диагональный элемент матрицы поворота равен
        R22 = 1 - 2*(x^2 + y^2),
    и это в точности косинус угла между осью Z тела и осью Z мира.

    Почему именно так, а не "крен больше порога": крен и тангаж по
    отдельности страдают от вырождения углов Эйлера и зависят от того,
    как робот развёрнут по рысканию. Угол наклона от вертикали такой
    зависимости не имеет и напрямую отвечает на вопрос "робот стоит на
    колёсах или лежит".

    0 градусов   — робот стоит ровно,
    90 градусов  — лежит на боку,
    180 градусов — лежит колёсами вверх.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0
    x, y = x / norm, y / norm
    cos_tilt = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.degrees(math.acos(cos_tilt))


class MowerStatusNode(Node):

    def __init__(self):
        super().__init__('mower_status_node')

        # ---------------- параметры ----------------
        # Имена топиков. В описании задания фигурировал /robot/gps, но в
        # реальных логах входной GPS называется иначе, поэтому имена
        # параметризованы и меняются без правки кода.
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('gps_topic', '/RoboMowerRealistic/gps')
        self.declare_parameter('speed_topic', '/RoboMowerRealistic/gps/speed')

        self.declare_parameter('status_topic', '/robot/status')
        self.declare_parameter('gps_out_topic', '/robot/gps')
        self.declare_parameter('gps_filtered_topic', '/robot/gps_filtered')

        # Фильтр Калмана.
        # gps_sigma — измеренное на неподвижном роботе СКО шума GPS, метры.
        # kf_process_density — спектральная плотность шума ускорения, м^2/с^3.
        self.declare_parameter('gps_sigma', 0.30)
        self.declare_parameter('kf_process_density', 0.10)

        # Детектор переворота. Гистерезис: включаемся на tilt_on,
        # выключаемся только ниже tilt_off, чтобы не дребезжать на границе.
        self.declare_parameter('tilt_on_deg', 80.0)
        self.declare_parameter('tilt_off_deg', 70.0)
        self.declare_parameter('tilt_hold_s', 0.3)

        # Детектор застревания.
        self.declare_parameter('speed_window_s', 0.5)
        self.declare_parameter('speed_on', 0.15)    # ниже этого — кандидат в застрявшие
        self.declare_parameter('speed_off', 0.25)   # выше этого — снова едет
        self.declare_parameter('stuck_hold_s', 1.0)

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('log_csv', '')       # путь для записи хода работы, '' = не писать

        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.tilt_on = float(g('tilt_on_deg'))
        self.tilt_off = float(g('tilt_off_deg'))
        self.tilt_hold = float(g('tilt_hold_s'))
        self.speed_window = float(g('speed_window_s'))
        self.speed_on = float(g('speed_on'))
        self.speed_off = float(g('speed_off'))
        self.stuck_hold = float(g('stuck_hold_s'))

        sigma = float(g('gps_sigma'))
        self.kf = Vector3KF(sigma * sigma, float(g('kf_process_density')))

        # ---------------- внутреннее состояние ----------------
        self.tilt_deg = 0.0
        self.have_imu = False
        self.have_gps = False
        self.have_speed = False

        self.speed_window_buf = deque()   # (t, скорость) за последние speed_window_s секунд
        self.last_gps_time = None
        self.raw_position = None
        self.filtered_position = None

        self.is_rotated = False
        self.rotated_since = None
        self.is_stuck = False
        self.stuck_since = None
        self.last_status = None

        # ---------------- подписки ----------------
        # QoS sensor_data (BEST_EFFORT) совместим и с BEST_EFFORT, и с
        # RELIABLE издателем. В логах топики записаны как RELIABLE, но с
        # таким профилем нода подключится в любом случае — обратный
        # вариант (RELIABLE подписчик к BEST_EFFORT издателю) не работает.
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointStamped, g('gps_topic'), self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Float32, g('speed_topic'), self.on_speed, qos_profile_sensor_data)

        # ---------------- публикации ----------------
        self.pub_status = self.create_publisher(String, g('status_topic'), 10)
        self.pub_gps = self.create_publisher(Vector3, g('gps_out_topic'), 10)
        self.pub_filtered = self.create_publisher(Vector3, g('gps_filtered_topic'), 10)

        # ---------------- журнал для графиков ----------------
        self.log_file = None
        self.log_writer = None
        log_path = str(g('log_csv'))
        if log_path:
            self.log_file = open(log_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                't', 'raw_x', 'raw_y', 'raw_z',
                'filt_x', 'filt_y', 'filt_z',
                'tilt_deg', 'mean_speed', 'status',
            ])
            self.get_logger().info(f'Пишу журнал в {log_path}')

        period = 1.0 / max(1e-3, float(g('publish_rate_hz')))
        self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Подписан на {g('imu_topic')}, {g('gps_topic')}, {g('speed_topic')}. "
            f'Порог переворота {self.tilt_on:.0f} град, порог застревания '
            f'{self.speed_on:.2f} м/с.'
        )

    # ------------------------------------------------------------------
    # Вспомогательное

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def mean_speed(self) -> float:
        if not self.speed_window_buf:
            return 0.0
        return sum(v for _, v in self.speed_window_buf) / len(self.speed_window_buf)

    # ------------------------------------------------------------------
    # Колбэки датчиков

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self.tilt_deg = tilt_from_quaternion(q.x, q.y, q.z, q.w)
        self.have_imu = True

    def on_gps(self, msg: PointStamped) -> None:
        t = self.now()
        p = msg.point

        # Сырые данные переупаковываем в Vector3, как просит задание.
        self.raw_position = (p.x, p.y, p.z)
        self.pub_gps.publish(Vector3(x=p.x, y=p.y, z=p.z))

        dt = 0.01 if self.last_gps_time is None else (t - self.last_gps_time)
        self.last_gps_time = t

        filtered, _velocity = self.kf.update(p.x, p.y, p.z, dt)
        self.filtered_position = filtered
        self.pub_filtered.publish(Vector3(x=filtered[0], y=filtered[1], z=filtered[2]))
        self.have_gps = True

    def on_speed(self, msg: Float32) -> None:
        t = self.now()
        self.speed_window_buf.append((t, float(msg.data)))
        # Выбрасываем всё, что старше окна усреднения.
        while self.speed_window_buf and self.speed_window_buf[0][0] < t - self.speed_window:
            self.speed_window_buf.popleft()
        self.have_speed = True

    # ------------------------------------------------------------------
    # Логика статуса

    def decide(self, t: float) -> str:
        # --- 1. Переворот. Проверяется первым: перевёрнутый робот тоже
        #        стоит на месте, и без приоритета он был бы помечен как
        #        застрявший. Для оператора "перевернулся" важнее.
        threshold = self.tilt_off if self.is_rotated else self.tilt_on
        if self.have_imu and self.tilt_deg > threshold:
            if self.rotated_since is None:
                self.rotated_since = t
            if t - self.rotated_since >= self.tilt_hold:
                self.is_rotated = True
        else:
            self.rotated_since = None
            self.is_rotated = False

        if self.is_rotated:
            # Сбрасываем счётчик застревания, чтобы после возврата робота
            # на колёса статус не прыгнул сразу в MOVEMENT_ON_PLACE.
            self.stuck_since = None
            self.is_stuck = False
            return STATUS_ROTATED

        # --- 2. Застревание.
        threshold = self.speed_off if self.is_stuck else self.speed_on
        if self.have_speed and self.mean_speed() < threshold:
            if self.stuck_since is None:
                self.stuck_since = t
            if t - self.stuck_since >= self.stuck_hold:
                self.is_stuck = True
        else:
            self.stuck_since = None
            self.is_stuck = False

        if self.is_stuck:
            return STATUS_STUCK

        # --- 3. Штатная езда.
        return STATUS_DRIVING

    def on_timer(self) -> None:
        # Пока не пришли данные хотя бы с одного датчика, молчим:
        # публиковать ROBOT DRIVING, ничего не измерив, было бы враньём.
        if not (self.have_imu or self.have_speed):
            return

        t = self.now()
        status = self.decide(t)

        self.pub_status.publish(String(data=status))

        if status != self.last_status:
            self.get_logger().info(
                f'Статус: {status}  (наклон {self.tilt_deg:.1f} град, '
                f'средняя скорость {self.mean_speed():.3f} м/с)'
            )
            self.last_status = status

        if self.log_writer is not None:
            raw = self.raw_position or (float('nan'),) * 3
            filt = self.filtered_position or (float('nan'),) * 3
            self.log_writer.writerow([
                f'{t:.6f}',
                f'{raw[0]:.6f}', f'{raw[1]:.6f}', f'{raw[2]:.6f}',
                f'{filt[0]:.6f}', f'{filt[1]:.6f}', f'{filt[2]:.6f}',
                f'{self.tilt_deg:.4f}', f'{self.mean_speed():.5f}', status,
            ])

    def destroy_node(self):
        if self.log_file is not None:
            self.log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MowerStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
