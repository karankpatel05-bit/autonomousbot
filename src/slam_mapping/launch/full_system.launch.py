#!/usr/bin/env python3
"""
full_system.launch.py  –  Master launch: SLAM Mapping + ESP32 motor bridge

Launches everything needed for autonomous mapping:
  • RPLidar A1M8 driver  (/dev/ttyUSB0)
  • SLAM Toolbox (async, map + odom frames)
  • Static TF chain
  • ESP32 motor bridge  (/dev/ttyUSB1, 4 × N20 encoder motors)
  • RViz2

Usage:
  ros2 launch auto full_system.launch.py

Override ports if needed:
  ros2 launch auto full_system.launch.py \
      lidar_port:=/dev/ttyUSB0 \
      esp32_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    slam_pkg = get_package_share_directory('slam_mapping')
    slam_yaml = os.path.join(slam_pkg, 'config', 'slam_params.yaml')
    rviz_cfg  = os.path.join(slam_pkg, 'config', 'mapping.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('lidar_port',      default_value='/dev/ttyUSB0',
                              description='USB port for RPLidar A1M8'),
        DeclareLaunchArgument('esp32_port',      default_value='/dev/ttyUSB1',
                              description='USB port for ESP32 SmallBot'),
        DeclareLaunchArgument('wheel_diameter',  default_value='0.150',
                              description='Wheel diameter (m)'),
        DeclareLaunchArgument('wheel_base',      default_value='0.400',
                              description='Distance between wheels (m)'),
        DeclareLaunchArgument('tpr_l',           default_value='349.0',
                              description='Ticks/rev LEFT wheel (measured)'),
        DeclareLaunchArgument('tpr_r',           default_value='362.0',
                              description='Ticks/rev RIGHT wheel (measured)'),
    ]

    return LaunchDescription(args + [

        # ── 1. RPLidar driver ─────────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      LaunchConfiguration('lidar_port'),
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'angle_compensate': True,
                'scan_mode':        'Standard',
                'scan_frequency':   10.0,
            }],
        ),

        # ── 2. Static TF: odom → base_footprint ──────────────────────────────
        # REMOVED: The esp32_bridge_node publishes this dynamically from
        # encoder odometry. A static publisher here would fight the dynamic
        # one and cause TF flicker / incorrect odometry.

        # ── 3. Static TF: base_footprint → laser ─────────────────────────────
        # z=0.305 — LiDAR is mounted 305 mm above robot base (new robot height).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            output='screen',
            arguments=['0.0','0.0','0.305','0','0','0','1','base_footprint','laser'],
        ),

        # ── 3a. Static TF: base_footprint → left_wheel ──────────────────────
        # Y = +wheel_base/2 = +0.200 m  (left side, wheel centre at body edge)
        # Z = -wheel_radius  = -0.0215 m (wheel centre below base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_left_wheel',
            output='screen',
            arguments=['0.0','0.200','-0.0215','0','0','0','1','base_footprint','left_wheel'],
        ),

        # ── 3b. Static TF: base_footprint → right_wheel ─────────────────────
        # Y = -wheel_base/2 = -0.200 m  (right side, wheel centre at body edge)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_right_wheel',
            output='screen',
            arguments=['0.0','-0.200','-0.0215','0','0','0','1','base_footprint','right_wheel'],
        ),

        # ── 4. SLAM Toolbox (delayed 3 s) ─────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_yaml],
                ),
            ],
        ),

        # ── 5. ESP32 SmallBot bridge (delayed 2 s for serial device to appear) ─
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='esp32_bridge',
                    executable='esp32_bridge_node',
                    name='esp32_bridge',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'port':           LaunchConfiguration('esp32_port'),
                        'baud':           115200,
                        'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                        'wheel_base':     LaunchConfiguration('wheel_base'),
                        'tpr_l':          LaunchConfiguration('tpr_l'),
                        'tpr_r':          LaunchConfiguration('tpr_r'),
                        'publish_rate':   20.0,
                        'cmd_timeout':    0.5,
                        'odom_frame':     'odom',
                        'base_frame':     'base_footprint',
                    }],
                ),
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
