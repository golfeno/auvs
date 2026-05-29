#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_auv_bringup')
    pkg_desc    = get_package_share_directory('my_auv_description')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.join(pkg_bringup, 'worlds', 'static_world.sdf')
    model_path  = os.path.join(pkg_desc,    'models', 'submarine.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn = TimerAction(period=4.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'submarine', '-file', model_path, '-x', '0', '-y', '0', '-z', '-2'],
             output='screen')
    ])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/model/submarine/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/submarine/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/model/submarine/joint/left_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/right_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder_top/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_front_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_front_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            # Балласты (режимы глубины 2=балласты / 3=оба)
            '/model/submarine/ballast_1/volume@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/ballast_2/volume@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/ballast_3/volume@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/ballast_4/volume@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        output='screen'
    )

    fake_baro = Node(
        package='my_auv_control', executable='fake_barometer',
        name='virtual_barometer', output='screen'
    )

    return LaunchDescription([gz_sim, spawn, bridge, fake_baro])
