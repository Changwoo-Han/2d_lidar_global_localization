import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import open3d as o3d
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ICP_Localizer(Node):
    def __init__(self):
        super().__init__('icp_localizer')
        self.map_cloud = None

        # QoS 설정 - scan은 best effort로 맞춰줌
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=scan_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=QoSProfile(depth=10))

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/icp_pose', 10)

    def map_callback(self, msg):
        self.get_logger().info('Received map message, converting to point cloud...')
        self.map_cloud = self.occupancygrid_to_pointcloud(msg)
        self.get_logger().info(f'Map point cloud created with {len(self.map_cloud.points)} points.')

    def occupancygrid_to_pointcloud(self, grid):
        points = []
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = grid.data

        occupied_count = 0  # 추가된 부분

        for y in range(height):
            for x in range(width):
                i = x + y * width
                if data[i] > 50:  # 점유 영역만
                    px = origin_x + x * resolution
                    py = origin_y + y * resolution
                    points.append([px, py, 0.0])
                    occupied_count += 1  # 추가된 부분

        self.get_logger().info(f'Occupied cells in map: {occupied_count}')  # 추가된 부분



        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc

    def scan_callback(self, scan_msg):
        self.get_logger().info('Scan callback triggered')
        if self.map_cloud is None:
            self.get_logger().warn('Map cloud is None, skipping ICP')
            return

        scan_points = self.laserscan_to_pointcloud(scan_msg)
        if len(scan_points.points) == 0:
            self.get_logger().warn('Empty scan points, skipping ICP')
            return

        threshold = 1.0
        trans_init = np.identity(4)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            scan_points, self.map_cloud, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        transformation = reg_p2p.transformation
        self.get_logger().info(f'ICP fitness: {reg_p2p.fitness:.3f}, inlier_rmse: {reg_p2p.inlier_rmse:.3f}')
        self.publish_pose(transformation)

    def laserscan_to_pointcloud(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y, 0.0])
            angle += scan_msg.angle_increment

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc

    def publish_pose(self, transformation):
        self.get_logger().info(f'Publishing pose: x={transformation[0,3]:.3f}, y={transformation[1,3]:.3f}')
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = transformation[0, 3]
        pose_msg.pose.pose.position.y = transformation[1, 3]
        pose_msg.pose.pose.position.z = 0.0

        yaw = math.atan2(transformation[1, 0], transformation[0, 0])
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        pose_msg.pose.covariance = [0.0] * 36

        self.pose_pub.publish(pose_msg)

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = ICP_Localizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
