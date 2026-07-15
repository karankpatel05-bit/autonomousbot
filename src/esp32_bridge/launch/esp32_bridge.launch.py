#!/usr/bin/env python3
"""
esp32_bridge.launch.py  —  SmallBot Arduino USB serial bridge

Protocol:
  RPI  → Arduino :  Single char commands: 'F', 'B', 'L', 'R', 'S', '1'-'5'
  Arduino → RPI  :  "E,<left>,<right>\n"  (encoder ticks @ 10 Hz)

Usage:
  ros2 launch esp32_bridge esp32_bridge.launch.py [arduino_port:=/dev/ttyUSB1]

Override measured wheel parameters if needed:
  ros2 launch esp32_bridge esp32_bridge.launch.py \
      arduino_port:=/dev/ttyUSB1 \
      tpr_l:=349.0 tpr_r:=362.0 \
      wheel_diameter:=0.043 wheel_base:=0.400

Direction fix (default -1.0 corrects Nav2 forward → robot backward bug):
  ros2 launch esp32_bridge esp32_bridge.launch.py linear_x_sign:=-1.0 angular_z_sign:=-1.0
  Use +1.0 if motors are wired in the opposite polarity.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('arduino_port',    default_value='/dev/ttyUSB1',
                              description='USB port for Arduino Uno motor base'),
        DeclareLaunchArgument('baud',            default_value='115200',
                              description='Serial baud rate'),
        DeclareLaunchArgument('wheel_diameter',  default_value='0.043',
                              description='Wheel diameter in metres (43 mm wheel)'),
        DeclareLaunchArgument('wheel_base',      default_value='0.400',
                              description='Distance between left and right wheel centres (m)'),
        DeclareLaunchArgument('tpr_l',           default_value='349.0',
                              description='Ticks per revolution — LEFT wheel (measured)'),
        DeclareLaunchArgument('tpr_r',           default_value='362.0',
                              description='Ticks per revolution — RIGHT wheel (measured)'),
        DeclareLaunchArgument('publish_rate',    default_value='10.0',
                              description='Odometry publish rate Hz (match Arduino 100 ms telemetry)'),
        DeclareLaunchArgument('cmd_timeout',     default_value='0.5',
                              description='Stop motors if no /cmd_vel within this many seconds'),
        DeclareLaunchArgument('linear_x_sign',   default_value='-1.0',
                              description='Direction correction: -1.0 = invert linear.x (fixes Nav2 going backward), +1.0 = pass through'),
        DeclareLaunchArgument('angular_z_sign',  default_value='-1.0',
                              description='Rotation correction: -1.0 = invert angular.z (fixes Nav2 left turn -> robot right turn)'),
    ]

    return LaunchDescription(args + [

        Node(
            package='esp32_bridge',
            executable='esp32_bridge_node',
            name='esp32_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port':           LaunchConfiguration('arduino_port'),
                'baud':           LaunchConfiguration('baud'),
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                'wheel_base':     LaunchConfiguration('wheel_base'),
                'tpr_l':          LaunchConfiguration('tpr_l'),
                'tpr_r':          LaunchConfiguration('tpr_r'),
                'publish_rate':   LaunchConfiguration('publish_rate'),
                'cmd_timeout':    LaunchConfiguration('cmd_timeout'),
                'odom_frame':     'odom',
                'base_frame':     'base_footprint',
                'linear_x_sign':  LaunchConfiguration('linear_x_sign'),
                'angular_z_sign': LaunchConfiguration('angular_z_sign'),
            }],
        ),

    ])
