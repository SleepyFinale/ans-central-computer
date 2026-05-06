#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import signal
import sys
import time


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PreciseUPattern(Node):

    def __init__(self):
        super().__init__('precise_u_pattern')

        # ======= PARAMETERS =======
        self.linear_speed = 0.2
        self.angular_speed = 0.6
        self.straight_distance = 0.75
        # ==========================

        # Update topic names when using a different robot namespace.
        self.publisher_ = self.create_publisher(Twist, '/inky/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/inky/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Phase tracking
        self.phase = 0
        self.start_x = None
        self.start_y = None

        # Rotation tracking
        self.prev_yaw = None
        self.accumulated_turn = 0.0

        self.get_logger().info("Running precise repeating U-pattern. Ctrl+C to stop safely.")

    # ===============================
    # ODOM CALLBACK
    # ===============================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ===============================
    # DISTANCE CALC
    # ===============================
    def distance_traveled(self):
        return math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

    # ===============================
    # ROTATION ACCUMULATION
    # ===============================
    def update_turn_accumulator(self):
        if self.prev_yaw is None:
            self.prev_yaw = self.current_yaw
            return

        delta = self.current_yaw - self.prev_yaw

        # Wrap to [-pi, pi]
        delta = math.atan2(math.sin(delta), math.cos(delta))

        self.accumulated_turn += abs(delta)
        self.prev_yaw = self.current_yaw

    def reset_turn_accumulator(self):
        self.prev_yaw = None
        self.accumulated_turn = 0.0

    # ===============================
    # MAIN CONTROL LOOP
    # ===============================
    def control_loop(self):

        msg = Twist()

        # ---- PHASE 0: STRAIGHT ----
        if self.phase == 0:
            if self.start_x is None:
                self.start_x = self.current_x
                self.start_y = self.current_y

            if self.distance_traveled() < self.straight_distance:
                msg.linear.x = self.linear_speed
            else:
                self.phase = 1
                self.reset_turn_accumulator()

        # ---- PHASE 1: U-TURN ----
        elif self.phase == 1:
            self.update_turn_accumulator()

            if self.accumulated_turn < math.pi:
                msg.linear.x = self.linear_speed
                msg.angular.z = self.angular_speed
            else:
                self.phase = 2
                self.start_x = self.current_x
                self.start_y = self.current_y

        # ---- PHASE 2: STRAIGHT BACK ----
        elif self.phase == 2:
            if self.distance_traveled() < self.straight_distance:
                msg.linear.x = self.linear_speed
            else:
                self.phase = 3
                self.reset_turn_accumulator()

        # ---- PHASE 3: U-TURN BACK ----
        elif self.phase == 3:
            self.update_turn_accumulator()

            if self.accumulated_turn < math.pi:
                msg.linear.x = self.linear_speed
                msg.angular.z = self.angular_speed
            else:
                self.phase = 0
                self.start_x = None  # restart cycle

        self.publisher_.publish(msg)

    # ===============================
    # SAFE STOP
    # ===============================
    def stop_robot(self):
        self.get_logger().info("Stopping robot safely...")

        stop_msg = Twist()
        for _ in range(10):
            self.publisher_.publish(stop_msg)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = PreciseUPattern()

    def signal_handler(sig, frame):
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
