"""Запуск ноды статуса.

Примеры:
    # только нода, лог проигрывается отдельным терминалом
    ros2 launch mower_status mower_status.launch.py

    # нода пишет журнал для построения графиков
    ros2 launch mower_status mower_status.launch.py log_csv:=/tmp/run_ezda.csv

    # нода и сразу воспроизведение лога
    ros2 launch mower_status mower_status.launch.py \
        bag:=/mnt/c/Users/Vladimir/Downloads/rosbag2_zastryal \
        log_csv:=/tmp/run_zastryal.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mower_status'), 'config', 'params.yaml')

    bag = LaunchConfiguration('bag')
    log_csv = LaunchConfiguration('log_csv')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='',
                              description='Путь к папке rosbag2. Пусто — лог не проигрывается.'),
        DeclareLaunchArgument('log_csv', default_value='',
                              description='Куда писать журнал работы для графиков.'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Скорость воспроизведения лога.'),

        Node(
            package='mower_status',
            executable='status_node',
            name='mower_status_node',
            output='screen',
            parameters=[params_file, {'log_csv': log_csv}],
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--rate', rate],
            output='screen',
            condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
        ),
    ])
