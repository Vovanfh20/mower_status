#!/usr/bin/env python3
"""Графики по журналу, который пишет нода (параметр log_csv).

Запуск:
    python3 scripts/plot_results.py /tmp/run_ezda.csv --title "Штатная езда" --out plots/ezda

Строит три фигуры:
    1. GPS по осям: сырой и отфильтрованный, с подсветкой статуса
    2. Сигналы детектора: угол наклона и средняя скорость с порогами
    3. Траектория в плане: видно, насколько фильтр убирает дрожание
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# Палитра: статусы — зарезервированные "состояния", не произвольные цвета серий.
C_RAW = '#8a8a85'        # сырые данные — приглушённые, это фон, а не ответ
C_FILT = '#2a78d6'       # отфильтрованные — основная серия
C_THRESH = '#52514e'     # линия порога — вспомогательная
C_GRID = '#d8d7d2'
C_TEXT = '#0b0b0b'
C_TEXT2 = '#52514e'

STATUS_COLOR = {
    'ROBOT DRIVING': '#008300',      # норма
    'MOVEMENT_ON_PLACE': '#eda100',  # предупреждение
    'ROTATED': '#e34948',            # авария
}
STATUS_RU = {
    'ROBOT DRIVING': 'ROBOT DRIVING (едет штатно)',
    'MOVEMENT_ON_PLACE': 'MOVEMENT_ON_PLACE (буксует)',
    'ROTATED': 'ROTATED (перевернулся)',
}


def read_log(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                rec = {k: (r[k] if k == 'status' else float(r[k])) for k in r}
            except ValueError:
                continue
            rows.append(rec)
    if not rows:
        raise SystemExit(f'В {path} нет данных')
    rows = trim_to_playback(rows)
    t0 = rows[0]['t']
    for r in rows:
        r['t'] -= t0
    return rows


def trim_to_playback(rows):
    """Оставляет только участок, где лог реально проигрывался.

    Нода работает дольше лога: её запускают заранее и гасят после. В эти
    промежутки GPS не обновляется, позиция стоит на месте, а статус
    уходит в MOVEMENT_ON_PLACE — на графике это пять минут пустоты вокруг
    девяти секунд полезного сигнала. Отрезаем по признаку "координата
    изменилась": первый и последний такой отсчёт и есть границы лога.
    """
    moving = [i for i, r in enumerate(rows)
              if math.isfinite(r['raw_x'])
              and (i == 0 or r['raw_x'] != rows[i - 1]['raw_x']
                   or r['raw_y'] != rows[i - 1]['raw_y'])]
    if len(moving) < 2:
        return rows
    first, last = moving[0], moving[-1]
    cut = rows[max(0, first - 1):last + 2]
    dropped = len(rows) - len(cut)
    if dropped > 0:
        print(f'  (отброшено {dropped} строк простоя до и после проигрывания лога)')
    return cut


def status_runs(rows):
    """Непрерывные интервалы одного статуса: [(t_нач, t_кон, статус), ...]."""
    runs = []
    for r in rows:
        if runs and runs[-1][2] == r['status']:
            runs[-1][1] = r['t']
        else:
            runs.append([r['t'], r['t'], r['status']])
    # Смыкаем интервалы встык, чтобы на графике не оставалось белых щелей.
    for cur, nxt in zip(runs, runs[1:]):
        cur[1] = nxt[0]
    return runs


def shade_status(ax, runs):
    for t0, t1, st in runs:
        ax.axvspan(t0, t1, color=STATUS_COLOR.get(st, '#cccccc'), alpha=0.10, lw=0)


def style(ax, ylabel):
    ax.set_ylabel(ylabel, color=C_TEXT2, fontsize=9)
    ax.grid(True, color=C_GRID, lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(C_GRID)
    ax.tick_params(colors=C_TEXT2, labelsize=8)


def status_legend(fig, runs):
    seen = [s for s in ('ROBOT DRIVING', 'MOVEMENT_ON_PLACE', 'ROTATED')
            if any(r[2] == s for r in runs)]
    handles = [Patch(facecolor=STATUS_COLOR[s], alpha=0.25, label=STATUS_RU[s]) for s in seen]
    fig.legend(handles=handles, loc='lower center', ncol=len(handles),
               frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.01))


def fig_axes(rows, runs, title, out):
    t = [r['t'] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, axis in zip(axes, 'xyz'):
        shade_status(ax, runs)
        ax.plot(t, [r[f'raw_{axis}'] for r in rows], color=C_RAW, lw=0.9, label='сырой GPS')
        ax.plot(t, [r[f'filt_{axis}'] for r in rows], color=C_FILT, lw=1.8,
                label='после фильтра Калмана')
        style(ax, f'{axis}, м')
    axes[0].legend(frameon=False, fontsize=8, loc='upper right')
    axes[-1].set_xlabel('время, с', color=C_TEXT2, fontsize=9)
    fig.suptitle(f'{title}: координаты GPS до и после фильтра', color=C_TEXT,
                 fontsize=12, y=0.97)
    status_legend(fig, runs)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(f'{out}_gps.png', dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def fig_detector(rows, runs, title, out, tilt_on, speed_on):
    t = [r['t'] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)

    shade_status(axes[0], runs)
    axes[0].plot(t, [r['tilt_deg'] for r in rows], color=C_FILT, lw=1.8)
    axes[0].axhline(tilt_on, color=C_THRESH, lw=1.2, ls='--')
    axes[0].annotate(f'порог {tilt_on:.0f}°', xy=(t[-1], tilt_on), xytext=(-4, 4),
                     textcoords='offset points', ha='right', fontsize=8, color=C_TEXT2)
    style(axes[0], 'наклон от вертикали, град')

    shade_status(axes[1], runs)
    axes[1].plot(t, [r['mean_speed'] for r in rows], color=C_FILT, lw=1.8)
    axes[1].axhline(speed_on, color=C_THRESH, lw=1.2, ls='--')
    axes[1].annotate(f'порог {speed_on:.2f} м/с', xy=(t[-1], speed_on), xytext=(-4, 4),
                     textcoords='offset points', ha='right', fontsize=8, color=C_TEXT2)
    style(axes[1], 'средняя скорость, м/с')
    axes[1].set_xlabel('время, с', color=C_TEXT2, fontsize=9)

    fig.suptitle(f'{title}: сигналы детектора и выставленный статус', color=C_TEXT,
                 fontsize=12, y=0.97)
    status_legend(fig, runs)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(f'{out}_detector.png', dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def fig_track(rows, title, out):
    fig, ax = plt.subplots(figsize=(6.5, 6))
    ax.plot([r['raw_x'] for r in rows], [r['raw_y'] for r in rows],
            color=C_RAW, lw=0.8, label='сырой GPS')
    ax.plot([r['filt_x'] for r in rows], [r['filt_y'] for r in rows],
            color=C_FILT, lw=2.0, label='после фильтра Калмана')
    ax.plot(rows[0]['filt_x'], rows[0]['filt_y'], 'o', ms=9, color='#1baf7a', label='старт')
    ax.plot(rows[-1]['filt_x'], rows[-1]['filt_y'], 's', ms=9, color='#e34948', label='финиш')
    style(ax, 'y, м')
    ax.set_xlabel('x, м', color=C_TEXT2, fontsize=9)
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(frameon=False, fontsize=8)
    ax.set_title(f'{title}: траектория в плане', color=C_TEXT, fontsize=12)
    fig.tight_layout()
    fig.savefig(f'{out}_track.png', dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def path_length(rows, prefix):
    """Пройденный путь. Строки без координат (GPS ещё не пришёл) пропускаем."""
    pts = []
    for r in rows:
        p = [r[f'{prefix}_{k}'] for k in 'xyz']
        if all(math.isfinite(v) for v in p):
            pts.append(p)
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--title', default='Лог')
    ap.add_argument('--out', default='plots/run')
    ap.add_argument('--tilt-on', type=float, default=80.0)
    ap.add_argument('--speed-on', type=float, default=0.15)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    rows = read_log(args.log)
    runs = status_runs(rows)

    fig_axes(rows, runs, args.title, args.out)
    fig_detector(rows, runs, args.title, args.out, args.tilt_on, args.speed_on)
    fig_track(rows, args.title, args.out)

    print(f'{args.title}')
    print(f'  длина пути по сырому GPS        : {path_length(rows, "raw"):7.2f} м')
    print(f'  длина пути после фильтра        : {path_length(rows, "filt"):7.2f} м')
    print('  интервалы статусов:')
    for t0, t1, st in runs:
        print(f'    {t0:6.2f} – {t1:6.2f} c  ({t1 - t0:5.2f} c)  {st}')
    print(f'  графики сохранены: {args.out}_gps.png, {args.out}_detector.png, '
          f'{args.out}_track.png')


if __name__ == '__main__':
    main()
