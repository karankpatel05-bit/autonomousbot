#!/usr/bin/env python3
"""
navigation.launch.py  —  SmallBot autonomous navigation on a saved map

Launches the full Nav2 stack:
  1. RPLidar A1M8 driver
  2. ESP32 SmallBot bridge (odometry + motor control)
  3. Static TF publishers (base→laser, base→left_wheel, base→right_wheel)
  4. Map server (loads saved map)
  5. AMCL (localization on saved map)
  6. Nav2 stack (planner, controller, smoother, behaviors, velocity smoother, BT navigator)
  7. Nav2 lifecycle manager
  8. RViz2 with navigation config

Usage:
  ros2 launch slam_mapping navigation.launch.py

  ros2 launch slam_mapping navigation.launch.py \\
      map:=/path/to/my_map.yaml \\
      lidar_port:=/dev/ttyUSB0 \\
      esp32_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('slam_mapping')
    nav2_yaml  = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_cfg   = os.path.join(pkg, 'config', 'navigation.rviz')
    default_map = os.path.join(os.path.expanduser('~'), 'auto', 'maps', 'smallbot_map.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('map',             default_value=default_map,
                              description='Full path to the saved map YAML file'),
        DeclareLaunchArgument('lidar_port',      default_value='/dev/ttyUSB0',
                              description='USB port for RPLidar A1M8'),
        DeclareLaunchArgument('esp32_port',      default_value='/dev/ttyUSB1',
                              description='USB port for ESP32 SmallBot'),
        DeclareLaunchArgument('wheel_diameter',  default_value='0.043',
                              description='Wheel diameter (m)'),
        DeclareLaunchArgument('wheel_base',      default_value='0.400',
                              description='Distance between wheels (m)'),
        DeclareLaunchArgument('tpr_l',           default_value='349.0',
                              description='Ticks/rev LEFT wheel'),
        DeclareLaunchArgument('tpr_r',           default_value='362.0',
                              description='Ticks/rev RIGHT wheel'),
    ]

    return LaunchDescription(args + [

        # ── 1. RPLidar A1M8 driver ────────────────────────────────────────────
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

        # ── 2. ESP32 SmallBot bridge (delayed 2 s) ───────────────────────────
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

        # ── 3. Static TF: base_footprint → laser ─────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            output='screen',
            arguments=['0.0', '0.0', '0.305', '0', '0', '0', '1',
                       'base_footprint', 'laser'],
        ),

        # ── 3a. Static TF: base_footprint → left_wheel ──────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_left_wheel',
            output='screen',
            arguments=['0.0', '0.200', '-0.0215', '0', '0', '0', '1',
                       'base_footprint', 'left_wheel'],
        ),

        # ── 3b. Static TF: base_footprint → right_wheel ─────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_right_wheel',
            output='screen',
            arguments=['0.0', '-0.200', '-0.0215', '0', '0', '0', '1',
                       'base_footprint', 'right_wheel'],
        ),

        # ── 4. Map Server ────────────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_yaml, {'yaml_filename': LaunchConfiguration('map')}],
        ),

        # ── 5. AMCL ──────────────────────────────────────────────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 6. Nav2 Planner Server ───────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 7. Nav2 Controller Server ────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 8. Nav2 Smoother Server ──────────────────────────────────────────
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 9. Nav2 Behavior Server (recoveries) ─────────────────────────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 10. Nav2 BT Navigator ────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 11. (Velocity smoother removed — ESP32 PID handles smooth accel) ─

        # ── 12. Nav2 Lifecycle Manager ───────────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── 13. RViz2 (delayed 4 s for Nav2 to initialize) ──────────────────
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_cfg],
                ),
            ],
        ),

    ])
