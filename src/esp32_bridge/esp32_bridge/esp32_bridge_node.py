#!/usr/bin/env python3
"""
esp32_bridge_node.py  —  ROS 2 ↔ SmallBot ESP32 serial bridge
==============================================================

Serial Protocol (115200 baud, USB CDC — NO rosserial):
  PC  → ESP32 :  "C:<v>,<omega>\\n"
                   v     : linear  velocity m/s  (float)
                   omega : angular velocity rad/s (float)
                   The ESP32 PID loop handles wheel speed regulation.

  ESP32 → PC  :  "E:<left_ticks>,<right_ticks>\\n"
                   Cumulative encoder ticks, sent at 20 Hz (every 50 ms).

  PC  → ESP32 :  "R\\n"   — reset encoder counts and PID integrators.

ROS Interfaces:
  Subscribes : /cmd_vel  (geometry_msgs/Twist)
  Publishes  : /odom     (nav_msgs/Odometry)
  Broadcasts : TF  odom → base_footprint

Odometry uses SEPARATE TPR per wheel (measured values from the real robot):
  dist_left  = (delta_l / tpr_l) * π * wheel_diameter
  dist_right = (delta_r / tpr_r) * π * wheel_diameter

Parameters (all overridable from launch):
  port           /dev/ttyUSB1
  baud           115200
  wheel_diameter 0.043        metres  (43 mm N20 wheel)
  wheel_base     0.140        metres  (140 mm between wheels)
  tpr_l          349.0        ticks/rev — LEFT  wheel (measured)
  tpr_r          362.0        ticks/rev — RIGHT wheel (measured)
  publish_rate   20.0         Hz
  cmd_timeout    0.5          seconds — stops motors if no /cmd_vel
  odom_frame     odom
  base_frame     base_footprint
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

try:
    import serial
except ImportError:
    raise ImportError(
        "pyserial not installed. Run: pip3 install pyserial  "
        "or: sudo apt install python3-serial"
    )


def _quat_from_yaw(yaw: float):
    """Return (x, y, z, w) quaternion for a pure Z-axis rotation."""
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


class ESP32BridgeNode(Node):
    """ROS 2 bridge for SmallBot ESP32 PID firmware over USB serial."""

    def __init__(self):
        super().__init__('esp32_bridge')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port',           '/dev/ttyUSB1')
        self.declare_parameter('baud',           115200)
        self.declare_parameter('wheel_diameter', 0.043)
        self.declare_parameter('wheel_base',     0.140)
        self.declare_parameter('tpr_l',          349.0)   # ticks/rev LEFT
        self.declare_parameter('tpr_r',          362.0)   # ticks/rev RIGHT
        self.declare_parameter('publish_rate',   20.0)
        self.declare_parameter('cmd_timeout',    0.5)
        self.declare_parameter('odom_frame',     'odom')
        self.declare_parameter('base_frame',     'base_footprint')
        self.declare_parameter('show_ticks',     True)   # print raw ticks to terminal

        port       = self.get_parameter('port').value
        baud       = int(self.get_parameter('baud').value)
        wheel_d    = float(self.get_parameter('wheel_diameter').value)
        self.L     = float(self.get_parameter('wheel_base').value)
        self.TPR_L = float(self.get_parameter('tpr_l').value)
        self.TPR_R = float(self.get_parameter('tpr_r').value)
        self.CIRC  = math.pi * wheel_d          # wheel circumference (m)
        rate_hz    = float(self.get_parameter('publish_rate').value)
        self.cmd_to= float(self.get_parameter('cmd_timeout').value)
        self.odom_fr  = self.get_parameter('odom_frame').value
        self.base_fr  = self.get_parameter('base_frame').value
        self.show_ticks = bool(self.get_parameter('show_ticks').value)

        # ── Open Serial ───────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud,
                timeout=1.0, write_timeout=1.0,
            )
            time.sleep(2.0)           # wait for ESP32 DTR reset + boot message
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(
                f'✓ SmallBot ESP32 connected on {port} @ {baud} baud'
            )
        except serial.SerialException as exc:
            self.get_logger().fatal(
                f'✗ Cannot open {port}: {exc}\n'
                '  Check: ls /dev/ttyUSB*'
            )
            raise

        # ── Internal State ────────────────────────────────────────────────────
        self._lock = threading.Lock()

        # Encoder snapshots (updated by reader thread from "E:" lines)
        self._ticks_l   = 0          # cumulative ticks, left
        self._ticks_r   = 0          # cumulative ticks, right
        self._prev_l    = 0
        self._prev_r    = 0
        self._enc_ready = False      # True after first E: packet
        self._tick_log_count = 0     # throttle counter for terminal tick prints

        # Odometry pose
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._vx  = 0.0
        self._vth = 0.0

        # Timestamps
        self._last_enc_time = self.get_clock().now()
        self._last_cmd_t    = time.monotonic()

        # ── ROS I/O ───────────────────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self._tf_bcast = tf2_ros.TransformBroadcaster(self)
        self._cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 10
        )

        # ── Reader thread + publish timer ─────────────────────────────────────
        self._running = True
        self._reader  = threading.Thread(
            target=self._serial_reader, daemon=True, name='esp32_reader'
        )
        self._reader.start()

        self.create_timer(1.0 / rate_hz, self._publish_odom)

        self.get_logger().info(
            f'SmallBot bridge ready | '
            f'wheel_circ={self.CIRC:.4f} m | base={self.L} m | '
            f'TPR L={self.TPR_L} R={self.TPR_R}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    #  /cmd_vel  →  "C:<v>,<omega>\n"
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: Twist):
        """Forward /cmd_vel as velocity command to ESP32 PID firmware."""
        v     = float(msg.linear.x)
        omega = float(msg.angular.z)

        # The ESP32 firmware computes differential drive internally:
        #   wheel_speed_L = v - omega * (WHEEL_BASE / 2)
        #   wheel_speed_R = v + omega * (WHEEL_BASE / 2)
        cmd = f'C:{v:.4f},{omega:.4f}\n'
        try:
            self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

        with self._lock:
            self._last_cmd_t = time.monotonic()

    def _send_stop(self):
        """Send zero-velocity command to ESP32."""
        try:
            self.ser.write(b'C:0.0000,0.0000\n')
        except serial.SerialException:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    #  Serial reader thread
    # ──────────────────────────────────────────────────────────────────────────
    def _serial_reader(self):
        buf = ''
        while self._running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    buf += self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                else:
                    time.sleep(0.004)
                    continue

                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._parse_line(line)

            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                time.sleep(0.5)
            except Exception as exc:
                self.get_logger().error(f'Reader error: {exc}')
                time.sleep(0.5)

    def _parse_line(self, line: str):
        """
        Parse lines from ESP32:
          "E:<left_ticks>,<right_ticks>"  — encoder report (20 Hz)
          Any other text is logged as debug.
        """
        if not line.startswith('E:'):
            self.get_logger().debug(f'ESP32: {line}')
            return

        # Strip "E:" prefix then split on ','
        body = line[2:]
        parts = body.split(',')
        if len(parts) != 2:
            self.get_logger().warning(f'Malformed E: packet: {line!r}')
            return

        try:
            l_ticks = int(parts[0])
            r_ticks = int(parts[1])
        except ValueError:
            self.get_logger().warning(f'Bad tick values: {line!r}')
            return

        with self._lock:
            if not self._enc_ready:
                self._prev_l    = l_ticks
                self._prev_r    = r_ticks
                self._enc_ready = True
                self.get_logger().info('✓ First E: packet — odometry active')

            self._ticks_l = l_ticks
            self._ticks_r = r_ticks

        # ── Throttled terminal display — once per second (every 20 packets) ──
        if self.show_ticks:
            self._tick_log_count += 1
            if self._tick_log_count >= 20:
                self._tick_log_count = 0
                # Use _prev_l/_prev_r which were set before this packet
                delta_l = l_ticks - self._prev_l
                delta_r = r_ticks - self._prev_r
                # Approximate speed: delta ticks in last ~1 s
                spd_l = (delta_l / self.TPR_L) * self.CIRC * 20.0
                spd_r = (delta_r / self.TPR_R) * self.CIRC * 20.0
                self.get_logger().info(
                    f'[TICKS]  L={l_ticks:+7d}  R={r_ticks:+7d}  '
                    f'\u0394L={delta_l:+5d}  \u0394R={delta_r:+5d}  '
                    f'vL={spd_l:+.3f} m/s  vR={spd_r:+.3f} m/s'
                )

    # ──────────────────────────────────────────────────────────────────────────
    #  Odometry timer
    # ──────────────────────────────────────────────────────────────────────────
    def _publish_odom(self):
        """Compute differential-drive odometry from encoder deltas → publish."""

        # ── cmd_vel watchdog ─────────────────────────────────────────────────
        if (time.monotonic() - self._last_cmd_t) > self.cmd_to:
            self._send_stop()

        # ── Snapshot ticks ───────────────────────────────────────────────────
        with self._lock:
            if not self._enc_ready:
                return
            cur_l = self._ticks_l
            cur_r = self._ticks_r

        delta_l = cur_l - self._prev_l
        delta_r = cur_r - self._prev_r
        self._prev_l = cur_l
        self._prev_r = cur_r

        # ── Time delta ───────────────────────────────────────────────────────
        now = self.get_clock().now()
        dt = (now - self._last_enc_time).nanoseconds * 1e-9
        self._last_enc_time = now

        if dt <= 0.0:
            return

        # ── Differential drive kinematics ─────────────────────────────────────
        # Each wheel has its own measured TPR  →  use separate conversions
        dist_l = (delta_l / self.TPR_L) * self.CIRC
        dist_r = (delta_r / self.TPR_R) * self.CIRC

        ds   = (dist_r + dist_l) / 2.0      # linear displacement
        dyaw = (dist_r - dist_l) / self.L   # heading change

        # Mid-point integration (more accurate than simple Euler)
        mid_yaw    = self._yaw + dyaw / 2.0
        self._x   += ds * math.cos(mid_yaw)
        self._y   += ds * math.sin(mid_yaw)
        self._yaw += dyaw

        self._vx  = ds   / dt
        self._vth = dyaw / dt

        # ── Build Odometry message ────────────────────────────────────────────
        qx, qy, qz, qw = _quat_from_yaw(self._yaw)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_fr
        odom.child_frame_id  = self.base_fr

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance[0]  = 0.01   # x variance
        odom.pose.covariance[7]  = 0.01   # y variance
        odom.pose.covariance[35] = 0.05   # yaw variance

        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.angular.z = self._vth
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[7]   = 0.01
        odom.twist.covariance[35]  = 0.05

        self._odom_pub.publish(odom)

        # ── Broadcast TF odom → base_footprint ───────────────────────────────
        tf = TransformStamped()
        tf.header.stamp            = now.to_msg()
        tf.header.frame_id         = self.odom_fr
        tf.child_frame_id          = self.base_fr
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = qx
        tf.transform.rotation.y    = qy
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf_bcast.sendTransform(tf)

    # ──────────────────────────────────────────────────────────────────────────
    #  Cleanup
    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        self._send_stop()
        try:
            self._reader.join(timeout=1.0)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
