#!/usr/bin/env python3
"""Извлечение IMU и GPS из rosbag2 (.db3) в CSV без зависимости от ROS2.

Нужен для офлайн-анализа: позволяет посмотреть данные и подобрать пороги,
не поднимая ROS и не читая гигабайты кадров камеры.

Файл .db3 — это обычная база SQLite с таблицей messages, где каждое
сообщение лежит сырым блобом в формате CDR. Скрипт берёт только нужные
топики и распаковывает байты по схеме сообщения.

Запуск:
    python3 scripts/extract_bag.py /путь/к/rosbag2_застрял
"""

import csv
import math
import os
import sqlite3
import sys


class CDR:
    """Минимальный разбор CDR с учётом выравнивания полей."""

    def __init__(self, buf: bytes):
        # Первые 4 байта — заголовок инкапсуляции; байт 1 задаёт порядок байт.
        self.little = buf[1] == 1
        self.buf = buf[4:]
        self.pos = 0

    def _align(self, size: int) -> None:
        self.pos = (self.pos + size - 1) // size * size

    def _read(self, fmt: str, size: int):
        import struct
        self._align(size)
        value = struct.unpack_from(('<' if self.little else '>') + fmt, self.buf, self.pos)[0]
        self.pos += size
        return value

    def uint32(self): return self._read('I', 4)
    def int32(self): return self._read('i', 4)
    def float32(self): return self._read('f', 4)
    def float64(self): return self._read('d', 8)

    def header(self) -> float:
        """std_msgs/Header: время + frame_id (строку пропускаем)."""
        sec, nanosec = self.int32(), self.uint32()
        self.pos += self.uint32()          # длина строки, затем сами байты
        return sec + nanosec * 1e-9


def quaternion_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


WANTED = {
    '/imu': 'imu',
    '/RoboMowerRealistic/gps': 'gps',
    '/RoboMowerRealistic/gps/speed': 'speed',
    '/RoboMowerRealistic/gps/speed_vector': 'speed_vector',
}

HEADERS = {
    'imu': ['t', 'roll', 'pitch', 'yaw', 'roll_deg', 'pitch_deg',
            'wx', 'wy', 'wz', 'ax', 'ay', 'az'],
    'gps': ['t', 'x', 'y', 'z'],
    'speed': ['t', 'speed'],
    'speed_vector': ['t', 'vx', 'vy', 'vz'],
}


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    bag_dir = sys.argv[1]

    db_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    if not db_files:
        raise SystemExit(f'В {bag_dir} нет файла .db3 — возможно, вложенная папка')

    con = sqlite3.connect(f'file:{os.path.join(bag_dir, db_files[0])}?mode=ro', uri=True)

    topics = {name: (tid, ttype) for tid, name, ttype
              in con.execute('SELECT id, name, type FROM topics')}
    print('Топики в логе:')
    for name, (_tid, ttype) in sorted(topics.items()):
        print(f'  {name:45s} {ttype}')

    ids = {topics[name][0]: short for name, short in WANTED.items() if name in topics}
    rows = {short: [] for short in ids.values()}

    print('\nЧитаю базу (кадры камеры пропускаются)...')
    query = ('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s)'
             % ','.join(map(str, ids)))
    for topic_id, stamp, blob in con.execute(query):
        kind = ids[topic_id]
        r = CDR(blob)
        if kind == 'imu':
            r.header()
            quat = [r.float64() for _ in range(4)]            # x, y, z, w
            [r.float64() for _ in range(9)]                   # orientation_covariance
            omega = [r.float64() for _ in range(3)]           # angular_velocity
            [r.float64() for _ in range(9)]                   # angular_velocity_covariance
            accel = [r.float64() for _ in range(3)]           # linear_acceleration
            roll, pitch, yaw = quaternion_to_rpy(*quat)
            rows[kind].append([stamp, roll, pitch, yaw,
                               math.degrees(roll), math.degrees(pitch)] + omega + accel)
        elif kind == 'gps':
            r.header()
            rows[kind].append([stamp, r.float64(), r.float64(), r.float64()])
        elif kind == 'speed':
            rows[kind].append([stamp, r.float32()])
        else:
            rows[kind].append([stamp, r.float64(), r.float64(), r.float64()])

    t0 = min(v[0][0] for v in rows.values() if v)
    for kind, values in rows.items():
        values.sort(key=lambda row: row[0])
        with open(f'{kind}.csv', 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(HEADERS[kind])
            for row in values:
                writer.writerow([f'{(row[0] - t0) * 1e-9:.6f}']
                                + [f'{v:.9g}' for v in row[1:]])
        print(f'{kind}.csv: {len(values)} строк')


if __name__ == '__main__':
    main()
