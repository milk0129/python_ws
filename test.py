import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image  # Pillow 라이브러리 사용
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        
        # QoS 설정
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # Use TRANSIENT_LOCAL durability
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

        # 빈 PGM 파일 생성 및 저장
        self.create_empty_pgm_file('/home/ksj/python_ws/empty_map.pgm', width=10, height=10)

        # 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Empty map publisher started.")

    def create_empty_pgm_file(self, filename, width=10, height=10):
        # 모든 값을 255 (흰색)으로 설정한 numpy 배열 생성
        data = np.full((height, width), 255, dtype=np.uint8)

        # PIL을 사용하여 PGM 이미지 생성 및 저장
        img = Image.fromarray(data, mode='L')  # 'L' 모드는 8-bit pixels, black and white
        img.save(filename)
        self.get_logger().info(f"Empty PGM file created: {filename}")

    def save_map_as_pgm(self, filename):
        img = Image.fromarray(self.map_data, mode='L')
        img.save(filename)
        self.get_logger().info(f"Updated map saved to {filename}")

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1.0  # Resolution of each cell (m)
        msg.info.width = 10
        msg.info.height = 10
        msg.info.origin.position.x = -5.0 
        msg.info.origin.position.y = -5.0
        msg.data = [-1] * (10 * 10)  # 모든 셀을 -1로 설정 (255에 해당하는 ROS의 "unknown" 상태)

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
