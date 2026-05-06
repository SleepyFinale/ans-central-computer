#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelWatermark(Node):
    def __init__(self):
        super().__init__('cmd_vel_watermark')

        # 1. Setup subscription and publisher.
        self.subscription = self.create_subscription(
            Twist,
            '/inky/cmd_vel_raw',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            '/inky/cmd_vel',
            10)

        # Publish watermark delay telemetry even when passthrough is active.
        self.delay_pub = self.create_publisher(
            Float64,
            '/inky/watermark_delay',
            10)

        # 2. Keep the same baseline value used by older experiments.
        self.mu = 0.050    # 50ms average network lag

        self.get_logger().info("Watermark passthrough active: publishing /cmd_vel immediately.")

    def listener_callback(self, msg):
        # 1. Zero out the delay math.
        delay = 0.0
        
        # 2. Keep telemetry topic populated for downstream consumers.
        delay_msg = Float64()
        delay_msg.data = float(0.05) - float(delay)
        self.delay_pub.publish(delay_msg)
        
        # 3. Publish directly (no queue, no timer delay).
        self.publisher.publish(msg)

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
