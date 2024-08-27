import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # Use TRANSIENT_LOCAL durability
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Empty map publisher started.")

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1.0  # Resolution of each cell (m)
        msg.info.width = 10
        msg.info.height = 10
        msg.info.origin.position.x = -5.0 
        msg.info.origin.position.y = -5.0
        msg.data = [0] * (10 * 10)  # Empty map (0 means unoccupied)

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing empty map.")

def main(args=None):
    rclpy.init(args=args)
    empty_map_publisher = EmptyMapPublisher()

    rclpy.spin(empty_map_publisher)
    empty_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
