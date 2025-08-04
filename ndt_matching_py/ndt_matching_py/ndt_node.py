import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_pointcloud')

        # LaserProjection 객체 생성
        self.laser_proj = LaserProjection()

        # BEST_EFFORT QoS 설정 (퍼블리셔와 호환성 위해)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = DurabilityPolicy.VOLATILE  # 꼭 추가하세요!


        # /scan 토픽 구독 (BEST_EFFORT)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile)

        # /input_cloud 토픽 퍼블리시 (BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, '/input_cloud', qos_profile)

    def scan_callback(self, scan_msg):
        # LaserScan -> PointCloud2 변환
        pc2_msg = self.laser_proj.projectLaser(scan_msg)

        # 변환된 PointCloud2 퍼블리시
        self.pub.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
