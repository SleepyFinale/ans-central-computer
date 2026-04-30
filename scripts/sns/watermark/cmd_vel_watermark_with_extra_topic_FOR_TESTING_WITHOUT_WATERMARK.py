#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64  # <-- ADDED: Import for the SOC delay topic
import random
from collections import deque
from rclpy.executors import ExternalShutdownException

class CmdVelWatermark(Node):
    def __init__(self):
        super().__init__('cmd_vel_watermark')

    # 1. Setup Subscription and Publisher
        # Note: cmd_vel usually uses default QoS, but we match your structure
        self.subscription = self.create_subscription(
            Twist,
            '/inky/cmd_vel_raw',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            '/inky/cmd_vel',
            10)

        # <-- ADDED: New Publisher for the SOC
        self.delay_pub = self.create_publisher(
            Float64,
            '/inky/watermark_delay',
            10)

    # 2. Gaussian Network Parameters
        self.mu = 0.050    # 50ms average network lag
        self.sigma = 0.020 # 20ms jitter 
        
    # 3. Buffer and Scheduler
        self.message_queue = deque()
        # Timer checks every 10ms (100Hz) to see if a packet is ready for release
        #self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f"Gaussian Watermark Active: Holding packets for ~{self.mu}s")

    def listener_callback(self, msg):
        # 1. Zero out the delay math
        delay = 0.0
        
        # 2. Update the SOC (0.05 - 0.0 = 0.05)
        delay_msg = Float64()
        delay_msg.data = float(0.05) - float(delay)
        self.delay_pub.publish(delay_msg)
        
        # 3. Bypass the queue and timer—publish DIRECTLY and IMMEDIATELY
        self.publisher.publish(msg)
        
        # Optional: Log to verify it's bypassing the queue
        # self.get_logger().info('Bypassing watermark: Immediate release')
    def timer_callback(self):
        # The "Network" releasing packets once their time has come
        now = self.get_clock().now()

        while self.message_queue and self.message_queue[0][0] <= now:
            release_time, msg = self.message_queue.popleft()
            self.publisher.publish(msg)
            
            # Optional: Log the delay to verify it's working
            # self.get_logger().info('Released delayed packet to /inky/cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatermark()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ignore the interrupt exception
        pass 
    finally:
        # 1. Destroy the node
        node.destroy_node()
        
        # 2. Try to shutdown, but silently ignore the "already shut down" error
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
