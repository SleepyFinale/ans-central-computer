#!/usr/bin/env python3
"""
Wait for required TF frames before starting Nav2.
This script waits for the complete TF tree: map -> odom -> base_link
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import sys
import time

class TFWaiter(Node):
    def __init__(self):
        super().__init__('tf_waiter')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.required_frames = [
            ('map', 'odom'),
            ('odom', 'base_link'),
        ]
        
        # Also check for base_footprint as alternative to base_link
        self.alternative_frames = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
        ]
        
    def check_tf_tree(self, frames_to_check):
        """Check if all required transforms are available"""
        all_ok = True
        for parent, child in frames_to_check:
            try:
                # Try to get the transform with a short timeout
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info(f"✓ Found transform: {parent} -> {child}")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                all_ok = False
                self.get_logger().debug(f"✗ Missing transform: {parent} -> {child} ({str(e)})")
        return all_ok
    
    def wait_for_tf(self, timeout=60):
        """Wait for TF tree to be ready"""
        self.get_logger().info("Waiting for TF tree to be ready...")
        self.get_logger().info("Required transforms:")
        self.get_logger().info("  map -> odom (from SLAM Toolbox)")
        self.get_logger().info("  odom -> base_link (from robot odometry)")
        self.get_logger().info("")
        
        start_time = time.time()
        check_interval = 0.5
        
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(f"Timeout after {timeout} seconds")
                return False
            
            # Check primary frames (base_link)
            if self.check_tf_tree(self.required_frames):
                self.get_logger().info("✓ TF tree is ready!")
                return True
            
            # Check alternative frames (base_footprint)
            if self.check_tf_tree(self.alternative_frames):
                self.get_logger().info("✓ TF tree is ready (using base_footprint)!")
                return True
            
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().info(f"Still waiting... ({int(elapsed)}s elapsed)")
            
            time.sleep(check_interval)
        
        return False

def main():
    rclpy.init()
    
    waiter = TFWaiter()
    
    try:
        # Spin once to initialize listeners
        rclpy.spin_once(waiter, timeout_sec=1.0)
        
        # Wait for TF tree
        success = waiter.wait_for_tf(timeout=60)
        
        if success:
            sys.exit(0)
        else:
            waiter.get_logger().error("TF tree not ready. Please check:")
            waiter.get_logger().error("  1. Robot launch is running: ros2 launch turtlebot3_bringup robot.launch.py")
            waiter.get_logger().error("  2. SLAM Toolbox is running: ros2 launch slam_toolbox online_async_launch.py")
            waiter.get_logger().error("  3. Wait 10-15 seconds after starting SLAM")
            sys.exit(1)
            
    except KeyboardInterrupt:
        waiter.get_logger().info("Interrupted by user")
        sys.exit(1)
    finally:
        waiter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
