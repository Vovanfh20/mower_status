"""Фильтр Калмана с моделью постоянной скорости для одной координаты.

Состояние x = [позиция, скорость]^T.

Модель движения (между соседними измерениями, шаг dt):
    p_k = p_{k-1} + v_{k-1} * dt
    v_k = v_{k-1} + w,     w ~ N(0, q * dt)

То есть робот считается движущимся равномерно, а всё отклонение от
равномерности (разгон, торможение, повороты) моделируется как белый шум
ускорения со спектральной плотностью q [м^2/с^3].

Измерение: z = p + n,  n ~ N(0, R).
R берётся из реально измеренного шума GPS (см. README): sigma ~ 0.3 м,
значит R = sigma^2 ~ 0.09 м^2.

Матрица шума процесса для модели постоянной скорости (стандартный вид):
    Q = q * [[dt^3/3, dt^2/2],
             [dt^2/2, dt   ]]
"""


class ConstantVelocityKF:
    """Скалярный фильтр Калмана: оценивает позицию и скорость по одной оси."""

    def __init__(self, measurement_variance: float, process_density: float):
        self.R = float(measurement_variance)   # дисперсия шума измерения, м^2
        self.q = float(process_density)        # спектральная плотность шума ускорения, м^2/с^3
        self.p = 0.0                           # оценка позиции, м
        self.v = 0.0                           # оценка скорости, м/с
        self.P = [[self.R, 0.0], [0.0, 1.0]]   # ковариация ошибки оценки
        self.initialised = False

    def reset(self) -> None:
        self.initialised = False
        self.p = 0.0
        self.v = 0.0
        self.P = [[self.R, 0.0], [0.0, 1.0]]

    def update(self, z: float, dt: float):
        """Один шаг фильтра: прогноз на dt секунд вперёд, затем коррекция по z."""
        # Первое измерение просто принимаем за начальную оценку.
        if not self.initialised:
            self.p = z
            self.v = 0.0
            self.P = [[self.R, 0.0], [0.0, 1.0]]
            self.initialised = True
            return self.p, self.v

        # Защита от нулевого и отрицательного шага (повторы меток времени в логе).
        if dt <= 0.0:
            dt = 1e-3

        # --- Прогноз ---
        self.p += self.v * dt

        P, q = self.P, self.q
        p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1] + q * dt ** 3 / 3.0
        p01 = P[0][1] + dt * P[1][1] + q * dt * dt / 2.0
        p10 = P[1][0] + dt * P[1][1] + q * dt * dt / 2.0
        p11 = P[1][1] + q * dt

        # --- Коррекция ---
        s = p00 + self.R                 # дисперсия невязки
        k0 = p00 / s                     # коэффициенты усиления Калмана
        k1 = p10 / s
        residual = z - self.p            # невязка измерения

        self.p += k0 * residual
        self.v += k1 * residual

        self.P = [
            [(1.0 - k0) * p00, (1.0 - k0) * p01],
            [p10 - k1 * p00, p11 - k1 * p01],
        ]
        return self.p, self.v


class Vector3KF:
    """Три независимых скалярных фильтра — по одному на оси x, y, z.

    Оси фильтруются раздельно, потому что шум GPS в логах изотропен и
    некоррелирован между осями (проверено по неподвижному участку:
    sigma_x = 0.34, sigma_y = 0.30, sigma_z = 0.21 м).
    """

    def __init__(self, measurement_variance: float, process_density: float):
        self.axes = [ConstantVelocityKF(measurement_variance, process_density)
                     for _ in range(3)]

    def reset(self) -> None:
        for axis in self.axes:
            axis.reset()

    def update(self, x: float, y: float, z: float, dt: float):
        """Возвращает (отфильтрованная позиция, оценка скорости) как два кортежа."""
        out = [axis.update(value, dt) for axis, value in zip(self.axes, (x, y, z))]
        position = tuple(o[0] for o in out)
        velocity = tuple(o[1] for o in out)
        return position, velocity
