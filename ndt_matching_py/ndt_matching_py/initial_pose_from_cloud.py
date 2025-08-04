import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from sklearn.cluster import KMeans
import math

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  
)

class InitialPoseEstimator(Node):
    def __init__(self):
        super().__init__('initial_pose_estimator')

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/input_cloud',
            self.pc_callback,
            qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile= qos
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.map_data = None
        self.has_published = False
        self.get_logger().info('InitialPoseEstimator 노드 시작')

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.get_logger().info("맵 데이터 수신됨!")

    def pc_callback(self, msg: PointCloud2):
        if self.has_published or self.map_data is None:
            return

        points = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        if not points:
            self.get_logger().warn('포인트클라우드에 점이 없습니다.')
            return

        # 거리 필터링
        xy_points = np.array([[p[0], p[1]] for p in points if 0.2 < math.hypot(p[0], p[1]) < 15.0])
        if len(xy_points) < 10:
            self.get_logger().warn('포인트 수가 너무 적습니다.')
            return

        n_clusters = min(10, max(3, len(xy_points) // 50))
        kmeans = KMeans(n_clusters=n_clusters, random_state=42)
        kmeans.fit(xy_points)
        centroids = kmeans.cluster_centers_

        def get_map_value(x, y):
            map_msg = self.map_data
            resolution = map_msg.info.resolution
            origin = map_msg.info.origin
            width = map_msg.info.width
            height = map_msg.info.height
            data = map_msg.data

            mx = int((x - origin.position.x) / resolution)
            my = int((y - origin.position.y) / resolution)

            if 0 <= mx < width and 0 <= my < height:
                idx = my * width + mx
                return data[idx]
            else:
                return -1

        def get_local_error(x, y):
            base = get_map_value(x, y)
            if base == -1:
                return 100

            neighbors = [
                get_map_value(x + dx, y + dy)
                for dx in [-0.1, 0, 0.1]
                for dy in [-0.1, 0, 0.1]
                if not (dx == 0 and dy == 0)
            ]
            valid_neighbors = [v for v in neighbors if v != -1]
            if not valid_neighbors:
                return base
            avg_neighbor = sum(valid_neighbors) / len(valid_neighbors)
            return 0.7 * base + 0.3 * avg_neighbor

        # 오차 계산 (occupancy 기반)
        errors = [get_local_error(cx, cy) for cx, cy in centroids]
        best_idx = np.argmin(errors)
        best_pos = centroids[best_idx]

        self.get_logger().info(f'Centroids: {centroids}')
        self.get_logger().info(f'Errors: {errors}')
        self.get_logger().info(f'Best position: {best_pos}')

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(best_pos[0])
        pose_msg.pose.pose.position.y = float(best_pos[1])
        pose_msg.pose.pose.position.z = 0.0

        # yaw = 0 고정
        pose_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        self.initialpose_pub.publish(pose_msg)
        self.get_logger().info('/initialpose 퍼블리시 완료')
        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
