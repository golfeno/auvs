#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_auv_bringup')
    pkg_desc    = get_package_share_directory('my_auv_description')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.join(pkg_bringup, 'worlds', 'static_world.sdf')
    model_path  = os.path.join(pkg_desc,    'models', 'submarine.sdf')
    rviz_cfg    = os.path.join(pkg_bringup, 'config', 'auv.rviz')

    use_rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
                                         description='Запускать RViz2 (true/false)')

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
            # TF: world -> submarine/body (динамическая поза аппарата) -> /tf
            '/model/submarine/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/submarine/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/model/submarine/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
            '/model/submarine/altimeter@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/submarine/sonar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/submarine/sonar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/model/submarine/joint/left_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/right_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder_top/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_front_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_front_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            # Балласты (режимы глубины 2=балласты / 3=оба)
            '/model/sub_ballast_1/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/sub_ballast_2/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/sub_ballast_3/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/sub_ballast_4/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double',
        ],
        remappings=[('/model/submarine/tf', '/tf')],
        output='screen'
    )

    fake_baro = Node(
        package='my_auv_control', executable='fake_barometer',
        name='virtual_barometer', output='screen'
    )

    # --- Статические TF: кадры сенсоров относительно корпуса (для отрисовки сканов) ---
    # Динамический world->submarine/body даёт OdometryPublisher (через bridge /tf).
    # frame_id сканов = "submarine/body/<sensor>" — публикуем их как дочерние к body.
    # TF совпадают с <pose> сенсоров в SDF (звено body повёрнуто +90° по Y).
    tf_sonar = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', '0.8', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'submarine/body', '--child-frame-id', 'submarine/body/sonar_sensor']
    )
    tf_alt = Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', '0.17', '--y', '0', '--z', '0',
                   '--frame-id', 'submarine/body', '--child-frame-id', 'submarine/body/altimeter_sensor']
    )

    # --- RViz2 (визуализация сонара/альтиметра/одометрии) ---
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_rviz, gz_sim, spawn, bridge, fake_baro,
        tf_sonar, tf_alt, rviz,
    ])
