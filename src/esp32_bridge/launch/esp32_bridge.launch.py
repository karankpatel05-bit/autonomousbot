#!/usr/bin/env python3
"""
esp32_bridge.launch.py  —  SmallBot ESP32 USB serial bridge

Protocol:
  PC  → ESP32 :  "C:<v>,<omega>\\n"    (velocity command)
  ESP32 → PC  :  "E:<left>,<right>\\n" (encoder ticks @ 20 Hz)

Usage:
  ros2 launch esp32_bridge esp32_bridge.launch.py [esp32_port:=/dev/ttyUSB1]

Override measured wheel parameters if needed:
  ros2 launch esp32_bridge esp32_bridge.launch.py \\
      esp32_port:=/dev/ttyUSB1 \\
      tpr_l:=349.0 tpr_r:=362.0 \\
      wheel_diameter:=0.043 wheel_base:=0.140
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('esp32_port',      default_value='/dev/ttyUSB1',
                              description='USB port for ESP32'),
        DeclareLaunchArgument('baud',            default_value='115200',
                              description='Serial baud rate'),
        DeclareLaunchArgument('wheel_diameter',  default_value='0.043',
                              description='Wheel diameter in metres (43 mm N20 wheel)'),
        DeclareLaunchArgument('wheel_base',      default_value='0.140',
                              description='Distance between left and right wheels (m)'),
        DeclareLaunchArgument('tpr_l',           default_value='349.0',
                              description='Ticks per revolution — LEFT wheel (measured)'),
        DeclareLaunchArgument('tpr_r',           default_value='362.0',
                              description='Ticks per revolution — RIGHT wheel (measured)'),
        DeclareLaunchArgument('publish_rate',    default_value='20.0',
                              description='Odometry publish rate Hz (match ESP32 loop = 20 Hz)'),
        DeclareLaunchArgument('cmd_timeout',     default_value='0.5',
                              description='Stop motors if no /cmd_vel within this many seconds'),
    ]

    return LaunchDescription(args + [

        Node(
            package='esp32_bridge',
            executable='esp32_bridge_node',
            name='esp32_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port':           LaunchConfiguration('esp32_port'),
                'baud':           LaunchConfiguration('baud'),
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                'wheel_base':     LaunchConfiguration('wheel_base'),
                'tpr_l':          LaunchConfiguration('tpr_l'),
                'tpr_r':          LaunchConfiguration('tpr_r'),
                'publish_rate':   LaunchConfiguration('publish_rate'),
                'cmd_timeout':    LaunchConfiguration('cmd_timeout'),
                'odom_frame':     'odom',
                'base_frame':     'base_footprint',
            }],
        ),

    ])
