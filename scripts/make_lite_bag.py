#!/usr/bin/env python3
"""Делает из тяжёлого bag лёгкую копию только с нужными топиками.

Зачем. Исходные логи весят 4–9 ГБ, и почти всё это кадры камеры и облака
точек. rosbag2 при проигрывании читает файл целиком независимо от ключа
--topics (фильтр работает на публикации, а не на чтении), поэтому через
медленный мост /mnt/c один прогон занимает минуты. SQLite с условием по
topic_id пропускает страницы с большими блобами и читает те же данные в
разы быстрее.

На выходе — обычный rosbag2 (sqlite3) на несколько мегабайт, который
проигрывается мгновенно и ничем не отличается от исходного для ноды.

Запуск:
    python3 make_lite_bag.py ~/bags/ezda ~/bags_lite/ezda
"""

import os
import sqlite3
import sys

import yaml

KEEP = [
    '/imu',
    '/RoboMowerRealistic/gps',
    '/RoboMowerRealistic/gps/speed',
    '/RoboMowerRealistic/gps/speed_vector',
]


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    src_dir, dst_dir = sys.argv[1], sys.argv[2]

    src_db_files = [f for f in os.listdir(src_dir) if f.endswith('.db3')]
    if not src_db_files:
        raise SystemExit(f'В {src_dir} нет .db3')
    src_db = os.path.join(src_dir, src_db_files[0])

    os.makedirs(dst_dir, exist_ok=True)
    dst_name = os.path.basename(dst_dir) + '_0.db3'
    dst_db = os.path.join(dst_dir, dst_name)
    if os.path.exists(dst_db):
        os.remove(dst_db)

    src = sqlite3.connect(f'file:{src_db}?mode=ro', uri=True)
    dst = sqlite3.connect(dst_db)

    # Переносим схему как есть, чтобы не угадывать версию формата rosbag2.
    for (sql,) in src.execute(
            "SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"):
        try:
            dst.execute(sql)
        except sqlite3.OperationalError:
            pass  # индексы и прочее, что может уже существовать

    # Таблица schema (версия формата) — копируем целиком, если она есть.
    try:
        rows = list(src.execute('SELECT * FROM schema'))
        n = len(rows[0]) if rows else 0
        if rows:
            dst.executemany(f'INSERT INTO schema VALUES ({",".join("?" * n)})', rows)
    except sqlite3.OperationalError:
        pass

    # Топики: берём только нужные, идентификаторы сохраняем прежними,
    # чтобы ссылки из messages остались корректными.
    cols = [d[1] for d in src.execute('PRAGMA table_info(topics)')]
    placeholders = ','.join('?' * len(cols))
    topics_src = list(src.execute(f'SELECT {",".join(cols)} FROM topics'))
    name_idx, id_idx = cols.index('name'), cols.index('id')
    type_idx = cols.index('type')
    qos_idx = cols.index('offered_qos_profiles') if 'offered_qos_profiles' in cols else None

    kept = [row for row in topics_src if row[name_idx] in KEEP]
    if not kept:
        raise SystemExit('Нужные топики в логе не найдены: '
                         + ', '.join(r[name_idx] for r in topics_src))
    dst.executemany(f'INSERT INTO topics ({",".join(cols)}) VALUES ({placeholders})', kept)

    keep_ids = [row[id_idx] for row in kept]
    print(f'{os.path.basename(src_dir)}: переношу {len(keep_ids)} топиков, читаю базу...')

    query = ('SELECT id, topic_id, timestamp, data FROM messages WHERE topic_id IN (%s)'
             % ','.join(map(str, keep_ids)))
    batch, total = [], 0
    counts = {tid: 0 for tid in keep_ids}
    t_min, t_max = None, None
    for mid, tid, stamp, blob in src.execute(query):
        batch.append((mid, tid, stamp, blob))
        counts[tid] += 1
        t_min = stamp if t_min is None else min(t_min, stamp)
        t_max = stamp if t_max is None else max(t_max, stamp)
        if len(batch) >= 5000:
            dst.executemany('INSERT INTO messages (id, topic_id, timestamp, data) '
                            'VALUES (?,?,?,?)', batch)
            total += len(batch)
            batch.clear()
    if batch:
        dst.executemany('INSERT INTO messages (id, topic_id, timestamp, data) '
                        'VALUES (?,?,?,?)', batch)
        total += len(batch)

    dst.commit()
    dst.close()
    src.close()

    duration = (t_max - t_min) if (t_min is not None) else 0

    meta = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': int(duration)},
            'starting_time': {'nanoseconds_since_epoch': int(t_min or 0)},
            'message_count': int(total),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': row[name_idx],
                        'type': row[type_idx],
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': (row[qos_idx] if qos_idx is not None else ''),
                    },
                    'message_count': int(counts[row[id_idx]]),
                }
                for row in kept
            ],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [dst_name],
            'files': [{
                'path': dst_name,
                'starting_time': {'nanoseconds_since_epoch': int(t_min or 0)},
                'duration': {'nanoseconds': int(duration)},
                'message_count': int(total),
            }],
        }
    }
    with open(os.path.join(dst_dir, 'metadata.yaml'), 'w') as fh:
        yaml.safe_dump(meta, fh, allow_unicode=True, default_flow_style=False, sort_keys=False)

    size_mb = os.path.getsize(dst_db) / 1e6
    print(f'  готово: {total} сообщений, {duration / 1e9:.2f} c, {size_mb:.1f} МБ -> {dst_dir}')


if __name__ == '__main__':
    main()
