#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pyproj import Proj, transform

class RawGpsPathNode(Node):
    def __init__(self):
        super().__init__('raw_gps_path_node')

        # --- 파라미터 ---
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
        self.declare_parameter('path_topic', '/raw_gps_path')
        self.declare_parameter('map_frame', 'map')
        
        # 건국대 일감호 좌표 (EKF 노드와 동일한 원점)
        self.declare_parameter('origin_lat', 37.540091)
        self.declare_parameter('origin_lon', 127.076555)

        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.map_frame_ = self.get_parameter('map_frame').get_parameter_value().string_value
        origin_lat = self.get_parameter('origin_lat').get_parameter_value().double_value
        origin_lon = self.get_parameter('origin_lon').get_parameter_value().double_value

        # --- 좌표 변환 설정 ---
        # WGS84 (위도/경도) -> UTM Zone 52N (EPSG:32652)
        # EKF 노드에서 사용하는 표준 UTM 투영법과 일치시킵니다.
        self.wgs84 = Proj(init='epsg:4326') # Lat/Lon
        self.utm = Proj(init='epsg:32652') # WGS 84 / UTM zone 52N
        
        # 원점 좌표를 UTM으로 변환
        self.origin_x_, self.origin_y_ = transform(self.wgs84, self.utm, origin_lon, origin_lat)
        self.origin_set_ = True

        self.get_logger().info(f"GPS 원점 설정 (일감호): LAT={origin_lat}, LON={origin_lon}")
        self.get_logger().info(f"변환된 UTM-K 원점: X={self.origin_x_:.2f}, Y={self.origin_y_:.2f}")

        # --- ROS I/O ---
        self.gps_sub_ = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            10)
        self.path_pub_ = self.create_publisher(Path, path_topic, 10)

        # --- 멤버 변수 ---
        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = self.map_frame_

        self.get_logger().info(f"'{path_topic}' 토픽으로 순수 GPS 경로를 발행합니다.")


    def gps_callback(self, msg: NavSatFix):
        if not self.origin_set_ or msg.status.status < msg.status.STATUS_FIX:
            # 원점이 설정되지 않았거나, GPS 수신 상태가 좋지 않으면 무시
            return

        # 현재 위치(위도/경도)를 UTM 좌표로 변환
        current_x, current_y = transform(self.wgs84, self.utm, msg.longitude, msg.latitude)

        # PoseStamped 메시지 생성
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame_

        # 원점 기준으로 상대 위치 계산
        pose.pose.position.x = current_x - self.origin_x_
        pose.pose.position.y = current_y - self.origin_y_
        pose.pose.position.z = 0.0  # Z값을 0으로 고정

        # 방향(orientation)은 알 수 없으므로 기본값 (0,0,0,1) 사용
        pose.pose.orientation.w = 1.0

        # 경로에 현재 위치 추가
        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.poses.append(pose)

        # 경로 메시지 발행
        self.path_pub_.publish(self.path_msg_)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RawGpsPathNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
