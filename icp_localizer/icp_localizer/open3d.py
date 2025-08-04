import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import open3d as o3d
import transforms3d.euler as t3d_euler
import transforms3d.quaternions as t3d_quat

class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_localizer')

        # 절대 경로로 map_path 설정
        self.declare_parameter("map_path", "/home/croft_robot/ros2_ws/src/icp_localizer/map.pcd")
        map_path = self.get_parameter("map_path").get_parameter_value().string_value

        self.map_pcd = o3d.io.read_point_cloud(map_path)
        if len(self.map_pcd.points) == 0:
            self.get_logger().error("Map cloud is empty or failed to load")
            return
        self.get_logger().info(f"Loaded map pointcloud with {len(self.map_pcd.points)} points")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/icp_pose', 10)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        zs = np.zeros_like(xs)
        scan_points = np.vstack((xs, ys, zs)).T

        if len(scan_points) < 10:
            self.get_logger().warn("Not enough scan points for ICP")
            return

        scan_pcd = o3d.geometry.PointCloud()
        scan_pcd.points = o3d.utility.Vector3dVector(scan_points)

        threshold = 1.0
        trans_init = np.identity(4)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            scan_pcd, self.map_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        if reg_p2p.fitness < 0.1:
            self.get_logger().warn("ICP fitness too low, skipping publish")
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        T = reg_p2p.transformation
        pose_msg.pose.pose.position.x = T[0, 3]
        pose_msg.pose.pose.position.y = T[1, 3]
        pose_msg.pose.pose.position.z = 0.0

        quat = t3d_quat.mat2quat(T[:3, :3])
        pose_msg.pose.pose.orientation.x = quat[1]
        pose_msg.pose.pose.orientation.y = quat[2]
        pose_msg.pose.pose.orientation.z = quat[3]
        pose_msg.pose.pose.orientation.w = quat[0]

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published ICP pose: ({T[0, 3]:.2f}, {T[1, 3]:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
