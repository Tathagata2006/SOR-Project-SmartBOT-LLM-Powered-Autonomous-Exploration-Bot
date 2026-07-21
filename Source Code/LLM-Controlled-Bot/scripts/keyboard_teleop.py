#!/usr/bin/env python3
"""
keyboard_teleop.py  —  Keyboard teleoperation for the diff-drive robot.

Your job: read keypresses from the terminal and publish geometry_msgs/Twist
messages on /cmd_vel so the robot moves accordingly.

Controls to implement
---------------------
  W / ↑   : forward
  S / ↓   : backward
  A / ←   : turn left
  D / →   : turn right
  Q        : forward-left arc
  E        : forward-right arc
  SPACE    : full stop (zero all velocity)
  + / =    : increase speed by 10 %
  - / _    : decrease speed by 10 %
  Ctrl-C   : quit cleanly, publish one final zero-velocity message

Stop-on-release semantics: the robot should stop when no recognised key
is pressed (treat any unknown key as a stop command).

ROS 2 parameters (already declared for you):
  ~linear_speed   (float, default 0.3)  m/s base forward/backward speed
  ~angular_speed  (float, default 0.8)  rad/s base turn speed
  ~publish_hz     (float, default 20.0) publish rate in Hz
  ~cmd_vel_topic  (str,   default /cmd_vel)

Hints
-----
- Use the `tty` and `termios` stdlib modules to read single keypresses
  without waiting for Enter.
- Arrow keys arrive as 3-byte ANSI escape sequences: ESC [ X
  (e.g. up arrow = '\\x1b[A').  Read the first byte; if it is '\\x1b',
  read two more bytes and concatenate.
- Run the ROS executor (rclpy.spin) in a background thread so the
  publish timer keeps firing while your key-reading loop blocks on stdin.
- Always restore terminal settings and publish a zero Twist on exit,
  even if an exception occurs — otherwise the terminal stays in raw mode.

Usage:
  ros2 run diff_drive_robot keyboard_teleop.py
  ros2 run diff_drive_robot keyboard_teleop.py --ros-args \
      -p linear_speed:=0.5 -p angular_speed:=1.2
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SPEED_STEP = 0.1   # fractional speed change per +/- keypress
SPEED_MIN  = 0.05
SPEED_MAX  = 2.0


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # ── parameters (do not change these) ────────────────────────────────
        self.declare_parameter('linear_speed',  0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_hz',    20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._lin_speed = self.get_parameter('linear_speed').value
        self._ang_speed = self.get_parameter('angular_speed').value
        hz              = self.get_parameter('publish_hz').value
        topic           = self.get_parameter('cmd_vel_topic').value

        # ── state ────────────────────────────────────────────────────────────
        self._lin_x: float = 0.0
        self._ang_z: float = 0.0
        self._lock  = threading.Lock()

        # ── ROS publisher + timer (do not change these) ───────────────────
        self._pub = self.create_publisher(Twist, topic, 10)
        self.create_timer(1.0 / hz, self._publish_cb)

        self.get_logger().info(
            f'keyboard_teleop ready  |  lin={self._lin_speed:.2f} m/s  '
            f'ang={self._ang_speed:.2f} rad/s  topic={topic}')

    # ── TODO 1: publish timer callback ───────────────────────────────────────
    def _publish_cb(self):
        """
        Publish the current velocity command as a Twist message.
        """
        msg = Twist()

        # Read the latest commanded velocities safely
        with self._lock:
            msg.linear.x = self._lin_x
            msg.angular.z = self._ang_z

        # Publish the velocity command
        self._pub.publish(msg)


    # ── TODO 2: velocity helpers ──────────────────────────────────────────────
    def _set_velocity(self, lin_factor: float, ang_factor: float):
        """
        Set the current commanded velocity.
        """
        with self._lock:
            self._lin_x = self._lin_speed * lin_factor
            self._ang_z = self._ang_speed * ang_factor

    def _stop(self):
        """
        Stop the robot.
        """
        with self._lock:
            self._lin_x = 0.0
            self._ang_z = 0.0

    def _change_speed(self, delta: float):
        """
        Increase or decrease both linear and angular speeds.
        """
        self._lin_speed = max(
            SPEED_MIN,
            min(SPEED_MAX, self._lin_speed * (1.0 + delta))
        )

        self._ang_speed = max(
            SPEED_MIN,
            min(SPEED_MAX, self._ang_speed * (1.0 + delta))
        )

        self.get_logger().info(
            f"Speed updated: linear={self._lin_speed:.2f} m/s, "
            f"angular={self._ang_speed:.2f} rad/s"
        )


    # ── TODO 3: key reading loop ──────────────────────────────────────────────

    def read_keys(self):
        """
        Blocking loop that reads keypresses and updates velocity state.
        """

        settings = termios.tcgetattr(sys.stdin)

        print("\n=== Keyboard Teleoperation ===")
        print("W/S : Forward/Backward")
        print("A/D : Left/Right")
        print("Q/E : Forward-Left / Forward-Right")
        print("SPACE : Stop")
        print("+/- : Increase/Decrease speed")
        print("Ctrl+C : Quit\n")

        try:
            while rclpy.ok():

                tty.setraw(sys.stdin.fileno())

                key = sys.stdin.read(1)

                if key == '\x1b':
                    key += sys.stdin.read(2)

                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    settings
                )

                if key == '\x03':          # Ctrl+C
                    break

                elif key == ' ':
                    self._stop()

                elif key in ['+', '=']:
                    self._change_speed(SPEED_STEP)

                elif key in ['-', '_']:
                    self._change_speed(-SPEED_STEP)

                elif key in ['w', 'W', '\x1b[A']:
                    self._set_velocity(1.0, 0.0)

                elif key in ['s', 'S', '\x1b[B']:
                    self._set_velocity(-1.0, 0.0)

                elif key in ['a', 'A', '\x1b[D']:
                    self._set_velocity(0.0, 1.0)

                elif key in ['d', 'D', '\x1b[C']:
                    self._set_velocity(0.0, -1.0)

                elif key in ['q', 'Q']:
                    self._set_velocity(1.0, 1.0)

                elif key in ['e', 'E']:
                    self._set_velocity(1.0, -1.0)

                else:
                    self._stop()

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                settings
            )

            self._stop()
            self._publish_cb()



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Spin ROS in a background thread so the timer keeps publishing
    # while the main thread blocks waiting for keypresses.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.read_keys()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
