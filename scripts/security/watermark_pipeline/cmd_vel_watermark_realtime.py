#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import random
from collections import deque
import threading
import subprocess
import time

class PureTSharkWatermark(Node):
    def __init__(self):
        super().__init__('pure_tshark_watermark')

        # 1. Pub/Sub
        self.subscription = self.create_subscription(
            Twist,
            '/inky/cmd_vel_raw',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/inky/cmd_vel', 10)
        self.delay_pub = self.create_publisher(Float64, '/inky/watermark_delay', 10)

        # 2. Parameters
        self.sigma = 0.020 
        self.current_tshark_delta = 0.050 # Initial fallback
        self.message_queue = deque()
        self.get_logger().info(f"Real-Time Chained Watermark Started. Sigma={self.sigma}s")        
        # 3. Threads/Timers
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.tshark_thread = threading.Thread(target=self.run_tshark, daemon=True)
        self.tshark_thread.start()

    def run_tshark(self):
        """Captures local packet deltas to drive the watermark"""
        # -i any: listen on all interfaces
        # -l: flush output immediately
        # -Y: display filter for ROS2/DDS RTPS packets
        tshark_cmd = [
            "tshark", "-i", "any", "-l", "-n",
            "-Y", "rtps.sm.wrEntityId == 0x00001203",
            "-T", "fields", "-e", "frame.time_delta_displayed"
        ]
        while rclpy.ok():
            try:
                proc = subprocess.Popen(tshark_cmd, stdout=subprocess.PIPE, text=True)
                for line in iter(proc.stdout.readline, ''):
                    val = line.strip()
                    if val:
                        try:
                            # Update watermark based on actual network arrival gaps
                            new_delta = float(val)
                            # Safety clamp: keep delay between 1ms and 500ms
                            self.current_tshark_delta = max(0.001, min(0.500, new_delta))
                            self.get_logger().info(f"Updated current_tshark_delta: {self.current_tshark_delta:.4f}s")
                        except ValueError:
                            continue
            except Exception as e:
                self.get_logger().error(f"TShark Subprocess Error: {e}")
                time.sleep(2)

    def listener_callback(self, msg):
        # Apply Gaussian noise to the TShark delta
        noise = random.gauss(0.0, self.sigma)
        total_delay = max(0.0, self.current_tshark_delta + noise)
        
        # Telemetry
        self.delay_pub.publish(Float64(data=float(total_delay)))
        
        # Queue for future release
        release_time = self.get_clock().now() + rclpy.duration.Duration(seconds=total_delay)
        self.message_queue.append((release_time, msg))

    def timer_callback(self):
        now = self.get_clock().now()
        while self.message_queue and self.message_queue[0][0] <= now:
            _, msg = self.message_queue.popleft()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PureTSharkWatermark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
