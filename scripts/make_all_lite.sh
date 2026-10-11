#!/usr/bin/env bash
# Делает лёгкие копии всех трёх логов в ~/bags_lite, затем прогоняет
# по ним ноду и собирает журналы в ~/mower_runs.
#
# Запуск:
#     bash ~/ros2_ws/src/mower_status/scripts/make_all_lite.sh

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

SCRIPTS="${HOME}/ros2_ws/src/mower_status/scripts"
LITE="${HOME}/bags_lite"
OUT="${HOME}/mower_runs"
mkdir -p "${LITE}" "${OUT}"

echo "### Шаг 1: лёгкие копии логов (один долгий проход по каждому файлу)"
for name in ezda zastryal perevernulsya; do
  if [ ! -f "${HOME}/bags/${name}/metadata.yaml" ]; then
    echo ">>> ПРОПУСК ${name}: нет ~/bags/${name}/metadata.yaml"
    continue
  fi
  if [ -f "${LITE}/${name}/metadata.yaml" ]; then
    echo ">>> ${name}: лёгкая копия уже есть, пропускаю"
    continue
  fi
  python3 "${SCRIPTS}/make_lite_bag.py" "${HOME}/bags/${name}" "${LITE}/${name}"
done

echo
echo "### Шаг 2: прогон ноды по лёгким копиям"
for name in ezda zastryal perevernulsya; do
  [ -f "${LITE}/${name}/metadata.yaml" ] || continue
  log="${OUT}/run_${name}.csv"
  echo
  echo "=================================================="
  echo ">>> ${name}  ->  ${log}"
  echo "=================================================="

  ros2 launch mower_status mower_status.launch.py "log_csv:=${log}" &
  launch_pid=$!
  sleep 4
  ros2 bag play "${LITE}/${name}"
  sleep 1
  pkill -INT -f status_node 2>/dev/null
  sleep 3; pkill -KILL -f mower_status.launch 2>/dev/null
  sleep 1

  if [ -s "${log}" ]; then
    echo ">>> ${name}: записано $(( $(wc -l < "${log}") - 1 )) строк"
  else
    echo ">>> ${name}: журнал пуст"
  fi
done

echo
echo "Готово."
wc -l "${OUT}"/*.csv
