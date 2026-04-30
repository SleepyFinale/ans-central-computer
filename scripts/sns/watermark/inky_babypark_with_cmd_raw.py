#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException # <--- IMPORTANT
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import subprocess # Add this at the top of your file

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class BabyParkOval(Node):
    def __init__(self):
        super().__init__('baby_park_oval')

        # ======= PARAMETERS =======
        self.linear_speed = 0.2 #0.2       
        self.angular_speed = 0.65 #0.65      
        self.straight_distance = 0.8  
        # ==========================

        self.publisher_ = self.create_publisher(Twist, '/inky/cmd_vel_raw', 10)
        self.subscription = self.create_subscription(Odometry, '/inky/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

# 1. Start at Phase 0
        self.phase = 0 
        
        # 2. Use None to indicate we don't have real data yet
        self.current_x = None 
        self.current_y = None
        self.current_yaw = 0.0
        
        self.start_x, self.start_y = None, None
        self.prev_yaw, self.accumulated_turn = None, 0.0

        self.odom_received = False
        self.get_logger().info("Inky Baby Park: Using simultaneous wheel speeds for curves.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True # Signal that we have data
    def control_loop(self):
# 1. Don't move until we actually hear where the robot is
        if not self.odom_received:
            return

        # 2. Capture the exact starting point of the first straight
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.prev_yaw = self.current_yaw
            self.get_logger().info("Starting Phase 0: Straight Line")
            return
        msg = Twist()

        # --- PHASE 0 & 2: STRAIGHT LINE ---
        if self.phase in [0, 2]:
            dist = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
            if dist < self.straight_distance:
                msg.linear.x = self.linear_speed
            else:
                self.get_logger().info("Straight finished. Starting Half-Circle.")
                self.phase += 1
                self.accumulated_turn = 0.0
                self.prev_yaw = self.current_yaw

        # --- PHASE 1 & 3: THE HALF-CIRCLE ---
        elif self.phase in [1, 3]:
            # 1. Calculate the change in angle since the last loop
            delta_yaw = self.current_yaw - self.prev_yaw
            
            # 2. Fix the "Wraparound" (if it jumps from 3.14 to -3.14)
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
            
            # 3. Add the absolute change to our counter
            self.accumulated_turn += abs(delta_yaw)
            self.prev_yaw = self.current_yaw

            # 4. Check if we have reached 180 degrees (pi radians)
            # PRO-TIP: Use 3.0 or 3.1 instead of math.pi to account for momentum
            if self.accumulated_turn < 2.9: 
                msg.linear.x = self.linear_speed 
                msg.angular.z = self.angular_speed
            else:
                self.get_logger().info("Half-Circle finished. Switching to Straight.")
                self.phase = (self.phase + 1) % 4
                # Reset straight-line starting point
                self.start_x, self.start_y = self.current_x, self.current_y

        self.publisher_.publish(msg)
    def stop_robot(self):
        print("\n[User Interrupt] Triggering System Emergency Stop...")
        try:
            # This runs the EXACT command you gave me in the terminal
            cmd = [
                'ros2', 'topic', 'pub', '--once', '/inky/cmd_vel_raw', 
                'geometry_msgs/msg/Twist', '{linear: {x: 0.0}, angular: {z: 0.0}}'
            ]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            print("System Stop Command Sent.")
        except Exception as e:
            print(f"Failed to run system stop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BabyParkOval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # We catch the Ctrl+C here
        pass 
    finally:
        # 1. Run the system command first (High Priority)
        node.stop_robot()
        
        # 2. Clean up the ROS node
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
