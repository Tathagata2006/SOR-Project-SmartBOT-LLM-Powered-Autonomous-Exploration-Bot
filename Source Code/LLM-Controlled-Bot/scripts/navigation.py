#!/usr/bin/env python3
"""
Custom obstacle-avoidance navigator (no Nav2 required).

All tuning values are ROS 2 parameters — override at launch:
  ros2 run diff_drive_robot navigation.py --ros-args \
      -p goal_x:=3.0 -p goal_y:=2.0 -p base_speed:=0.8
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class ReliableObstacleNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')

        self.declare_parameter('goal_x',             5.0)
        self.declare_parameter('goal_y',             4.0)
        self.declare_parameter('obstacle_threshold', 0.9)
        self.declare_parameter('clearance_required', 1.5)
        self.declare_parameter('move_distance',      1.0)
        self.declare_parameter('scan_angle_deg',     60.0)
        self.declare_parameter('front_angle_deg',    30.0)
        self.declare_parameter('base_speed',         0.8)
        self.declare_parameter('turn_speed',         2.0)
        self.declare_parameter('goal_tolerance',     0.3)
        self.declare_parameter('timer_period',       0.05)
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('odom_topic',     '/odom')

        self.goal = [
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
        ]
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.clearance_required = self.get_parameter('clearance_required').value
        self.move_distance      = self.get_parameter('move_distance').value
        self.scan_angle         = math.radians(self.get_parameter('scan_angle_deg').value)
        self.front_angle_range  = math.radians(self.get_parameter('front_angle_deg').value)
        self.base_speed         = self.get_parameter('base_speed').value
        self.turn_speed         = self.get_parameter('turn_speed').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        odom_topic    = self.get_parameter('odom_topic').value

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.state      = 'GOAL_SEEK'
        self.robot_pos  = [0.0, 0.0, 0.0]   # x, y, yaw
        self.start_pos  = [0.0, 0.0]
        self.target_yaw = 0.0
        self.laser_ranges: list = []
        self.laser_angles: list = []

        timer_period = self.get_parameter('timer_period').value
        self.create_timer(timer_period, self.navigate)

        self.get_logger().info(
            f'Navigator ready. Goal: ({self.goal[0]}, {self.goal[1]})')

    # ------------------------------------------------------------------
    # Callbacks — do not modify
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pos[2] = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2))

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if len(self.laser_angles) != len(msg.ranges):
            self.laser_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))]

    def distance_moved(self):
        return math.hypot(
            self.robot_pos[0] - self.start_pos[0],
            self.robot_pos[1] - self.start_pos[1])

    # ------------------------------------------------------------------
    # TODO 1 — Front obstacle distance
    # ------------------------------------------------------------------

    def get_front_obstacle_distance(self):
        """
        Return the minimum lidar distance directly in front of the robot.
        """
        if not self.laser_ranges:
            return float('inf')

        front_ranges = []

        for r, angle in zip(self.laser_ranges, self.laser_angles):
            if abs(angle) <= self.front_angle_range:
                if math.isfinite(r):
                    front_ranges.append(r)

        if not front_ranges:
            return float('inf')

        return min(front_ranges)

    # ------------------------------------------------------------------
    # TODO 2 — Clear direction search
    # ------------------------------------------------------------------

    def find_clear_direction(self):
        """
        Find a direction that is both clear and as close as possible
        to the goal heading.
        """
        goal_yaw = math.atan2(
            self.goal[1] - self.robot_pos[1],
            self.goal[0] - self.robot_pos[0]
        )

        if not self.laser_ranges:
            return False, goal_yaw

        best_score = -float('inf')
        best_absolute_yaw = goal_yaw

        sector_width = math.radians(10)

        for heading in np.arange(-math.pi / 2,
                                 math.pi / 2 + sector_width,
                                 sector_width):

            sector = []

            for r, a in zip(self.laser_ranges, self.laser_angles):
                if abs(a - heading) <= sector_width / 2:
                    if math.isfinite(r):
                        sector.append(r)

            if not sector:
                continue

            clearance = min(sector)

            if clearance < self.clearance_required:
                continue

            absolute_heading = self.robot_pos[2] + heading

            goal_error = math.atan2(
                math.sin(goal_yaw - absolute_heading),
                math.cos(goal_yaw - absolute_heading)
            )

            alignment = math.cos(goal_error)

            score = clearance + 2.0 * alignment

            if score > best_score:
                best_score = score
                best_absolute_yaw = absolute_heading

        if best_score == -float('inf'):
            return False, goal_yaw

        return True, best_absolute_yaw

    # ------------------------------------------------------------------
    # TODO 3 — Navigation FSM
    # ------------------------------------------------------------------

    def navigate(self):
        twist = Twist()

        goal_dx = self.goal[0] - self.robot_pos[0]
        goal_dy = self.goal[1] - self.robot_pos[1]

        goal_distance = math.hypot(goal_dx, goal_dy)
        goal_yaw = math.atan2(goal_dy, goal_dx)

        yaw_error = goal_yaw - self.robot_pos[2]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if goal_distance <= self.goal_tolerance:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Goal reached!")
            self.destroy_node()
            return

        front_distance = self.get_front_obstacle_distance()

        if self.state == 'GOAL_SEEK':

            if front_distance < self.obstacle_threshold:
                self.state = 'FIND_CLEAR'
                return

            alignment = max(
                0.0,
                1.0 - abs(yaw_error) / math.radians(40)
            )

            twist.linear.x = self.base_speed * alignment
            twist.angular.z = 1.2 * yaw_error

        elif self.state == 'FIND_CLEAR':

            found, self.target_yaw = self.find_clear_direction()

            yaw_error = self.target_yaw - self.robot_pos[2]
            yaw_error = math.atan2(
                math.sin(yaw_error),
                math.cos(yaw_error)
            )

            twist.angular.z = 1.2 * yaw_error

            if abs(yaw_error) < math.radians(5):
                self.start_pos = self.robot_pos[:2]
                self.state = 'MOVE_CLEAR'

        elif self.state == 'MOVE_CLEAR':

            yaw_error = self.target_yaw - self.robot_pos[2]
            yaw_error = math.atan2(
                math.sin(yaw_error),
                math.cos(yaw_error)
            )

            if front_distance < self.obstacle_threshold:

                found, self.target_yaw = self.find_clear_direction()

                if found:
                    self.state = 'FIND_CLEAR'
                else:
                    twist.linear.x = -0.3
                    twist.angular.z = self.turn_speed * 0.5

            elif self.distance_moved() >= self.move_distance:

                self.state = 'REALIGN'

            else:

                twist.linear.x = self.base_speed * 0.8
                twist.angular.z = 1.2 * yaw_error

        elif self.state == 'REALIGN':

            yaw_error = goal_yaw - self.robot_pos[2]
            yaw_error = math.atan2(
                math.sin(yaw_error),
                math.cos(yaw_error)
            )

            twist.angular.z = 1.2 * yaw_error

            if abs(yaw_error) < math.radians(5):
                self.state = 'GOAL_SEEK'

        # Clamp velocities
        twist.linear.x = max(
            -self.base_speed,
            min(self.base_speed, twist.linear.x)
        )

        twist.angular.z = max(
            -self.turn_speed,
            min(self.turn_speed, twist.angular.z)
        )

        # Guard against NaN
        if not math.isfinite(twist.linear.x):
            twist.linear.x = 0.0

        if not math.isfinite(twist.angular.z):
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ReliableObstacleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
