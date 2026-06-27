#!/usr/bin/env python3
"""cmd_vel -> Wave Rover bridge with dead-reckoning odometry and tank-drive joystick.

Subscribes to /cmd_vel (geometry_msgs/Twist), converts to differential-drive
left/right motor values in [-1, 1], and sends them to the rover as JSON
({"T":1,"L":..,"R":..}). The rover firmware accepts the identical command set
over either transport:

    transport: http    -> HTTP over WiFi   (http://<rover_ip>/js?json=...)
    transport: serial  -> USB/UART @115200 (writes the JSON line to a tty)

Also:
  - Publishes dead-reckoning /odom and broadcasts odom->base_link TF.
    Integrates *commanded* velocities (no encoders), so pair with slam_toolbox
    to correct drift.
  - Subscribes to /joy for tank-drive teleoperation.  While the deadman button
    is held the left stick Y controls the left wheels and the right stick Y
    controls the right wheels directly.  Releasing the deadman hands control
    back to cmd_vel (Nav2 or teleop_twist_joy).

Run with:
    ros2 run wave_rover_controller wave_rover_bridge.py
or via the launch file (loads config/wave_rover_bridge.yaml):
    ros2 launch wave_rover_controller wave_rover_bridge_launch.py
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import tf2_ros


class HttpTransport:
    """Send JSON commands over HTTP (WiFi).

    Uses `requests` if installed, otherwise falls back to the stdlib `urllib`
    so it works on the QIR image with no extra packages.
    """

    name = 'http'

    def __init__(self, node):
        self._ip = node.get_parameter('rover_ip').value
        self._timeout = node.get_parameter('request_timeout').value
        self._session = None
        try:
            import requests
            self._session = requests.Session()
            self._send_impl = self._send_requests
            backend = 'requests'
            self.exceptions = (requests.exceptions.RequestException,)
        except ImportError:
            import urllib.request
            self._urlopen = urllib.request.urlopen
            self._send_impl = self._send_urllib
            backend = 'urllib'
            # urllib.error.URLError and socket.timeout both subclass OSError.
            self.exceptions = (OSError,)
        self.summary = f'http://{self._ip} (timeout {self._timeout}s, {backend})'

    def _send_requests(self, cmd):
        self._session.get(
            f'http://{self._ip}/js?json={cmd}', timeout=self._timeout)

    def _send_urllib(self, cmd):
        self._urlopen(
            f'http://{self._ip}/js?json={cmd}', timeout=self._timeout)

    def send(self, cmd):
        self._send_impl(cmd)

    def close(self):
        if self._session is not None:
            self._session.close()


class _RawSerial:
    """Minimal write-only serial port via termios (pyserial fallback).

    Used when pyserial is not installed (e.g. the QIR Yocto image). We only ever
    write newline-terminated JSON, so a raw fd in 8N1 raw mode is sufficient.
    """

    def __init__(self, port, baud):
        import os
        import termios
        baud_const = getattr(termios, f'B{baud}', None)
        if baud_const is None:
            raise ValueError(f'unsupported baud_rate {baud} for termios fallback')
        self._os = os
        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0  # iflag: no input processing
        attrs[1] = 0  # oflag: no output processing
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0  # lflag: raw (no canonical/echo)
        attrs[4] = baud_const  # ispeed
        attrs[5] = baud_const  # ospeed
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)

    def write(self, data):
        self._os.write(self._fd, data)

    def close(self):
        self._os.close(self._fd)


class SerialTransport:
    """Send JSON commands over USB/UART (pyserial, falling back to termios)."""

    name = 'serial'

    def __init__(self, node):
        self._port = node.get_parameter('serial_port').value
        self._baud = node.get_parameter('baud_rate').value
        try:
            import serial
            # timeout=0 -> non-blocking writes; we never read back.
            self._ser = serial.Serial(self._port, self._baud, timeout=0)
            backend = 'pyserial'
            self.exceptions = (serial.SerialException, OSError)
        except ImportError:
            self._ser = _RawSerial(self._port, self._baud)
            backend = 'termios'
            self.exceptions = (OSError,)
        self.summary = f'{self._port} @ {self._baud} ({backend})'

    def send(self, cmd):
        self._ser.write((cmd + '\n').encode('ascii'))

    def close(self):
        self._ser.close()


TRANSPORTS = {t.name: t for t in (HttpTransport, SerialTransport)}


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('waverover_bridge')

        # Transport: 'http' (WiFi) or 'serial' (USB/UART).
        self.declare_parameter('transport', 'http')

        # HTTP params.
        # rover_ip: 192.168.4.1 in AP mode (rover's own hotspot), or the
        # router-assigned IP shown on the OLED when in STA mode.
        self.declare_parameter('rover_ip', '192.168.4.1')
        # Per-command HTTP timeout, seconds.
        self.declare_parameter('request_timeout', 0.2)

        # Serial params.
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Diff-drive tuning.
        self.declare_parameter('track_width', 0.15)
        self.declare_parameter('max_speed', 1.0)
        # Safety watchdog: if no cmd_vel arrives within this many seconds, send a
        # stop. Set to 0.0 to disable.
        self.declare_parameter('cmd_timeout', 0.5)

        # Odometry frames.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Set False if something else (e.g. robot_localization) broadcasts this TF.
        self.declare_parameter('publish_tf', True)

        # Tank-drive joystick.
        # Axis indices into sensor_msgs/Joy.axes[].  The scale flips sign:
        # most controllers report stick-up as -1, so scale = -1.0 gives
        # positive = forward.  Set joy_deadman_button to -1 to require no button.
        self.declare_parameter('joy_left_axis', 1)       # left stick Y
        self.declare_parameter('joy_left_scale', -1.0)
        self.declare_parameter('joy_right_axis', 3)      # right stick Y
        self.declare_parameter('joy_right_scale', -1.0)
        self.declare_parameter('joy_deadman_button', 5)  # RB (matches existing teleop)

        self.track    = self.get_parameter('track_width').value
        self.max_spd  = self.get_parameter('max_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self._odom_frame  = self.get_parameter('odom_frame').value
        self._base_frame  = self.get_parameter('base_frame').value
        self._publish_tf  = self.get_parameter('publish_tf').value

        self._joy_left_axis   = self.get_parameter('joy_left_axis').value
        self._joy_left_scale  = self.get_parameter('joy_left_scale').value
        self._joy_right_axis  = self.get_parameter('joy_right_axis').value
        self._joy_right_scale = self.get_parameter('joy_right_scale').value
        self._joy_deadman_btn = self.get_parameter('joy_deadman_button').value

        self.add_on_set_parameters_callback(self._on_params)

        transport = self.get_parameter('transport').value
        if transport not in TRANSPORTS:
            raise ValueError(
                f"Unknown transport '{transport}', "
                f"expected one of {sorted(TRANSPORTS)}")
        self.transport = TRANSPORTS[transport](self)

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Joy,   'joy',     self._joy_cb,     10)

        # Watchdog: stop rover if cmd_vel goes silent and joy is not active.
        self._last_cmd_time = self.get_clock().now()
        self._stopped = True
        self._joy_active = False
        if self.cmd_timeout > 0.0:
            self.create_timer(self.cmd_timeout / 2.0, self._watchdog_cb)

        # Dead-reckoning odometry state.
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_th  = 0.0
        self._odom_vx  = 0.0
        self._odom_vth = 0.0
        self._last_odom_time = None

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Publish odom at 20 Hz so TF stays fresh even when the rover is idle.
        self.create_timer(0.05, self._odom_timer_cb)

        self.get_logger().info(
            f'waverover_bridge ready: transport={self.transport.name} '
            f'-> {self.transport.summary} | '
            f'track_width={self.track} max_speed={self.max_spd} '
            f'cmd_timeout={self.cmd_timeout}s | '
            f'odom {self._odom_frame}->{self._base_frame} '
            f'publish_tf={self._publish_tf} | '
            f'tank-drive axes L={self._joy_left_axis} R={self._joy_right_axis} '
            f'deadman_btn={self._joy_deadman_btn}')

    # ── parameter callback ────────────────────────────────────────────────────

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'max_speed':
                self.max_spd = p.value
                self.get_logger().info(f'max_speed -> {self.max_spd}')
            elif p.name == 'track_width':
                self.track = p.value
                self.get_logger().info(f'track_width -> {self.track}')
        return SetParametersResult(successful=True)

    # ── cmd_vel (Nav2 / teleop_twist_joy) ────────────────────────────────────

    def _cmd_vel_cb(self, msg):
        # Always update the timestamp so the watchdog knows cmd_vel is alive,
        # even while the joystick has priority.
        self._last_cmd_time = self.get_clock().now()
        self._stopped = (abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3)

        if self._joy_active:
            return  # joystick has priority; ignore the motion command

        x = msg.linear.x
        z = msg.angular.z

        self._integrate_odom(x, z)

        left  = (x - z * self.track / 2.0) / self.max_spd
        right = (x + z * self.track / 2.0) / self.max_spd
        self._send(left, right)

    # ── tank-drive joystick ───────────────────────────────────────────────────

    def _joy_cb(self, msg):
        deadman = (
            self._joy_deadman_btn < 0
            or (self._joy_deadman_btn < len(msg.buttons)
                and msg.buttons[self._joy_deadman_btn])
        )

        if not deadman:
            if self._joy_active:
                self._joy_active = False
                self._stopped = True
                self._odom_vx  = 0.0
                self._odom_vth = 0.0
                self._send(0.0, 0.0)
                self.get_logger().info('Tank drive released — cmd_vel resumed')
            return

        if not self._joy_active:
            self.get_logger().info('Tank drive active')
        self._joy_active = True

        left  = self._axis(msg, self._joy_left_axis,  self._joy_left_scale)
        right = self._axis(msg, self._joy_right_axis, self._joy_right_scale)

        # Derive vx / vth from the per-wheel speeds for odom integration.
        # (Inverse of the cmd_vel -> L/R formula.)
        vx  = (left + right) / 2.0 * self.max_spd
        vth = (right - left) / self.track

        self._integrate_odom(vx, vth)
        self._send(left, right)

    def _axis(self, msg, idx, scale):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx]) * scale

    # ── watchdog ─────────────────────────────────────────────────────────────

    def _watchdog_cb(self):
        if self._stopped or self._joy_active:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn('cmd_vel timeout — stopping rover')
            self._stopped = True
            self._odom_vx  = 0.0
            self._odom_vth = 0.0
            self._send(0.0, 0.0)

    # ── odometry ──────────────────────────────────────────────────────────────

    def _integrate_odom(self, vx, vth):
        now = self.get_clock().now()
        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9
            self._odom_x  += vx * math.cos(self._odom_th) * dt
            self._odom_y  += vx * math.sin(self._odom_th) * dt
            self._odom_th += vth * dt
        self._odom_vx  = vx
        self._odom_vth = vth
        self._last_odom_time = now

    def _odom_timer_cb(self):
        now = self.get_clock().now()

        q = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self._odom_th / 2.0),
            w=math.cos(self._odom_th / 2.0),
        )

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame
            t.child_frame_id  = self._base_frame
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame
        odom.pose.pose.position.x  = self._odom_x
        odom.pose.pose.position.y  = self._odom_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = self._odom_vx
        odom.twist.twist.angular.z = self._odom_vth

        # Diagonal pose covariance [x, y, z, rx, ry, yaw].
        # Open-loop dead reckoning — slam_toolbox corrects the drift.
        odom.pose.covariance[0]  = 0.1   # x   (m²)
        odom.pose.covariance[7]  = 0.1   # y   (m²)
        odom.pose.covariance[35] = 0.2   # yaw (rad²)
        # Diagonal twist covariance [vx, vy, vz, vrx, vry, vyaw].
        odom.twist.covariance[0]  = 0.05  # vx   (m/s)²
        odom.twist.covariance[35] = 0.1   # vyaw (rad/s)²

        self._odom_pub.publish(odom)

    # ── motor output ──────────────────────────────────────────────────────────

    def _send(self, left, right):
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        cmd = f'{{"T":1,"L":{left:.3f},"R":{right:.3f}}}'
        self.get_logger().debug(f'-> {cmd}')
        try:
            self.transport.send(cmd)
        except self.transport.exceptions as e:
            self.get_logger().warn(
                f'Send failed: {e}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.transport.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
