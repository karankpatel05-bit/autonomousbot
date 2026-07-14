#!/usr/bin/env python3
"""
slam.launch.py  –  RPLidar A1M8 + SLAM Toolbox lidar mapping

Nodes launched:
  1. rplidar_ros  (rplidar_composition)    on /dev/ttyUSB0
  2. static_transform_publisher            map → odom (identity, until EKF)
  3. static_transform_publisher            odom → base_footprint (identity)
  4. static_transform_publisher            base_footprint → laser (sensor offset)
  5. slam_toolbox async_slam_toolbox_node  (delayed 3 s)
  6. rviz2                                 with mapping.rviz

Usage:
  ros2 launch slam_mapping slam.launch.py [lidar_port:=/dev/ttyUSB0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slam_mapping')
    slam_yaml = os.path.join(pkg, 'config', 'slam_params.yaml')
    rviz_cfg  = os.path.join(pkg, 'config', 'mapping.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='USB serial port for the RPLidar A1M8'
    )
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([

        lidar_port_arg,

        # ── 1. RPLidar A1M8 driver ────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      lidar_port,
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'angle_compensate': True,
                'scan_mode':        'Standard',
                'scan_frequency':   10.0,
            }],
        ),

        # ── 2. Static TF: map → odom (identity until EKF is added) ───────────
        # When running SLAM standalone (no EKF), SLAM Toolbox itself publishes
        # map→odom. This publisher is a safety fallback only.
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_map_odom',
        #     arguments=['0','0','0','0','0','0','1','map','odom'],
        # ),

        # ── 3. Static TF: odom → base_footprint (zero offset) ────────────────
        # SLAM Toolbox publishes odom→base_footprint when use_odometry=false,
        # but we still need a placeholder so TF tree is complete from startup.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base',
            output='screen',
            arguments=[
                '0', '0', '0',        # x y z
                '0', '0', '0', '1',   # qx qy qz qw
                'odom', 'base_footprint'
            ],
        ),

        # ── 4. Static TF: base_footprint → laser (LiDAR mount offset) ────────
        # z=0.305 — LiDAR mounted 305 mm above base (new robot height 305mm).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            output='screen',
            arguments=[
                '0.0', '0.0', '0.305',  # x y z  – LiDAR is 30.5 cm above base
                '0',   '0',   '0', '1',   # qx qy qz qw – no rotation
                'base_footprint', 'laser'
            ],
        ),

        # ── 4a. Static TF: base_footprint → left_wheel ──────────────────────
        # Y = +0.200 m (half wheel_base, left side — 400mm wheel base)
        # Z = -0.0215 m (wheel centre below base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_left_wheel',
            output='screen',
            arguments=[
                '0.0', '0.200', '-0.0215',
                '0', '0', '0', '1',
                'base_footprint', 'left_wheel'
            ],
        ),

        # ── 4b. Static TF: base_footprint → right_wheel ─────────────────────
        # Y = -0.200 m (half wheel_base, right side — 400mm wheel base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_right_wheel',
            output='screen',
            arguments=[
                '0.0', '-0.200', '-0.0215',
                '0', '0', '0', '1',
                'base_footprint', 'right_wheel'
            ],
        ),

        # ── 5. SLAM Toolbox  (delayed 3 s so LiDAR scan topic is live) ───────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_yaml],
                    remappings=[
                        ('/scan', '/scan'),
                    ],
                )
            ],
        ),

        # ── 6. RViz2 ──────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),

    ])
