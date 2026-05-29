#!/usr/bin/env python3
"""Запуск мира + бридж + спаун submarine_nb (без балластов)"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

M = 'submarine_nb'

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_auv_bringup')
    pkg_desc    = get_package_share_directory('my_auv_description')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.join(pkg_bringup, 'worlds', 'static_world.sdf')
    model_path  = os.path.join(pkg_desc, 'models', f'{M}.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn = TimerAction(period=4.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', M, '-file', model_path, '-x', '0', '-y', '0', '-z', '-2'],
             output='screen')
    ])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            f'/model/{M}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/{M}/joint/left_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/{M}/joint/right_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/{M}/joint/vertical_rudder/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/{M}/joint/vertical_rudder_top/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/{M}/joint/horizontal_rudder_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/{M}/joint/horizontal_rudder_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, spawn, bridge])
