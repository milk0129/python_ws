import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import numpy as np
from PIL import Image
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import math

class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        
        # QoS 설정
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

        # TF 브로드캐스터 초기화
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 빈 PGM 파일 생성 및 저장
        self.map_data = np.full((10, 10), 255, dtype=np.int8)  # 초기 맵 데이터 생성 (모든 셀을 -1로 초기화)
        self.create_empty_pgm_file('/home/ksj/python_ws/empty_map.pgm', self.map_data)

        # 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Empty map publisher started.")

    def create_empty_pgm_file(self, filename, map_data):
        # P2 형식의 PGM 파일 생성 및 저장
        with open(filename, 'w') as f:
            f.write("P2\n")  # P2 형식
            f.write(f"{map_data.shape[1]} {map_data.shape[0]}\n")  # 너비와 높이
            f.write("255\n")  # 최대 색상 값
            for row in map_data:
                f.write(" ".join(map(str, row + 1)) + "\n")  # -1은 0으로 변환하여 저장
        self.get_logger().info(f"Empty PGM file created: {filename}")

    def save_map_as_pgm(self, filename):
        # 업데이트된 맵 데이터를 P2 형식의 PGM 파일로 저장
        with open(filename, 'w') as f:
            f.write("P2\n")  # P2 형식
            f.write(f"{self.map_data.shape[1]} {self.map_data.shape[0]}\n")  # 너비와 높이
            f.write("255\n")  # 최대 색상 값
            for row in self.map_data:
                f.write(" ".join(map(str, row + 1)) + "\n")  # -1은 0으로 변환하여 저장
        self.get_logger().info(f"Updated map saved to {filename}")


    def gps_to_grid(self, latitude, longitude, map_width, map_height, resolution, origin_x, origin_y):
        # 위도와 경도를 맵의 그리드 좌표로 변환
        x = int((longitude - origin_x) / resolution + map_width / 2)
        y = int((latitude - origin_y) / resolution + map_height / 2)
        return x, y

    def publish_transform(self, x, y, theta):
        # TF 메시지 생성 및 전송
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(x)  # int -> float 변환
        t.transform.translation.y = float(y)  # int -> float 변환
        t.transform.translation.z = 0.0

        q = self.euler_to_quaternion(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasting TF from map to base_link at ({x}, {y}, {theta})")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def timer_callback(self):
        # 임의의 GPS 좌표를 사용하여 그리드 좌표 계산
        latitude = 37.727681
        longitude = 127.061285
        x, y = self.gps_to_grid(latitude, longitude, 10, 10, 1.0, -5.0, -5.0)

        # 계산된 그리드 좌표를 사용하여 맵 데이터 업데이트
        if 0 <= x < 10 and 0 <= y < 10:
            self.map_data[y, x] = 0  # 해당 위치를 0으로 설정 (ROS에서 0은 unoccupied 상태를 의미)

        # OccupancyGrid 메시지 생성
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1.0
        msg.info.width = 10
        msg.info.height = 10
        msg.info.origin.position.x = -5.0 
        msg.info.origin.position.y = -5.0

        # 맵 데이터를 ROS OccupancyGrid 메시지에 맞게 변환
        msg.data = self.map_data.flatten().tolist()

        # 맵 데이터 및 TF 브로드캐스트
        self.publisher_.publish(msg)
        self.publish_transform(x, y, 0.0)

        # 업데이트된 맵을 PGM 파일로 저장
        self.save_map_as_pgm('/home/ksj/python_ws/updated_map.pgm')

        self.get_logger().info(f"Publishing updated map with GPS point at ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    empty_map_publisher = EmptyMapPublisher()
    rclpy.spin(empty_map_publisher)
    empty_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
