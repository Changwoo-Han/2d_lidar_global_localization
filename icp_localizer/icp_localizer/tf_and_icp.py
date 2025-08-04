import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import transforms3d.euler as t3d_euler
import transforms3d.quaternions as t3d_quat
import random

class ICPWithGTNode(Node):
    def __init__(self):
        super().__init__('icp_with_gt_node')

        # Map pcd 경로 파라미터
        self.declare_parameter("map_path", "/home/croft_robot/ros2_ws/src/croft_auto/croft_auto_nav2/config/my_map3.pcd")
        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        self.map_pcd = o3d.io.read_point_cloud(map_path)
        if not self.map_pcd.has_points():
            self.get_logger().error(f"Failed to load map pointcloud or empty: {map_path}")
        else:
            self.get_logger().info(f"Loaded map pointcloud with {len(self.map_pcd.points)} points")

        self.robot_name = 'croft_auto'

        # Gazebo model_states_demo 구독 (최신 GT 위치 계속 받음)
        self.create_subscription(ModelStates, '/model_states_demo', self.gazebo_pose_callback, 10)

        # 입력 pointcloud 구독 (QoS BEST_EFFORT로 맞춤)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/input_cloud', self.pointcloud_callback, qos)

        # 초기 위치 퍼블리셔 (RViz 초기 pose 설정용)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.ground_truth_pose = None  # 최신 GT 변환행렬 (4x4)
        self.initial_gt_pose = None    # 최초 1회 GT 변환행렬 (초기 ICP 역변환 계산용)
        self.icp_transform = np.eye(4) # ICP 누적 변환행렬

        # ICP 초기값 (랜덤 위치 범위 내에서 세팅)
        self.icp_init = None

    def gazebo_pose_callback(self, msg: ModelStates):
        try:
            index = msg.name.index(self.robot_name)
            pose = msg.pose[index]
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            q = pose.orientation
            quat = [q.w, q.x, q.y, q.z]
            rot = t3d_quat.quat2mat(quat)

            gt_transform = np.eye(4)
            gt_transform[:3, :3] = rot
            gt_transform[:3, 3] = [x, y, z]

            self.ground_truth_pose = gt_transform

            if self.initial_gt_pose is None:
                # 최초 1회만 저장
                self.initial_gt_pose = gt_transform
                self.get_logger().info(f"[GT] 최초 1회 GT 위치 저장: ({x:.2f}, {y:.2f}, {z:.2f})")

                # 최초 1회 랜덤 위치 범위 기반 ICP 초기값 생성
                # 랜덤 위치 범위 내에서 초기 위치 세팅
                rand_x = random.uniform(-5.0, 5.0)
                rand_y = random.uniform(-5.0, 5.0)
                rand_yaw = random.uniform(-3.14, 3.14)
                rot_init = t3d_euler.euler2mat(0, 0, rand_yaw)
                T_init = np.eye(4)
                T_init[:3, :3] = rot_init
                T_init[0, 3] = rand_x
                T_init[1, 3] = rand_y
                self.icp_init = T_init
                self.get_logger().info(f"[ICP INIT] 랜덤 초기 위치 설정: x={rand_x:.2f}, y={rand_y:.2f}, yaw={rand_yaw:.2f}")

            else:
                # 최초 이후 최신 GT 위치 정보 로그
                gt_pos = gt_transform[:3, 3]
                self.get_logger().debug(f"[GT] 최신 위치: {gt_pos.round(2)}")

        except ValueError:
            self.get_logger().warn(f"{self.robot_name} not found in /model_states_demo")

    def pointcloud_callback(self, msg: PointCloud2):
        if self.ground_truth_pose is None or self.icp_init is None:
            return  # GT 위치와 ICP 초기값 준비 안됨

        scan_pcd = self.convert_ros_to_o3d(msg)
        if scan_pcd.is_empty():
            self.get_logger().warn("Received empty scan")
            return

        threshold = 1.5

        icp_result = o3d.pipelines.registration.registration_icp(
            scan_pcd, self.map_pcd, threshold,
            self.icp_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        self.icp_transform = icp_result.transformation

        fitness = icp_result.fitness
        rmse = icp_result.inlier_rmse
        self.get_logger().info(f"[ICP] Fitness: {fitness:.3f}, RMSE: {rmse:.4f}")

        # ICP 추정 위치 (T_map_base 역행렬로 base_map 추정)
        T_icp_inv = np.linalg.inv(self.icp_transform)
        est_position = T_icp_inv[:3, 3]

        # 최신 GT 위치와 비교
        gt_position = self.ground_truth_pose[:3, 3]
        distance_error = np.linalg.norm(est_position - gt_position)

        self.get_logger().info(f"[EVAL] Estimated: {est_position.round(2)}, Ground Truth: {gt_position.round(2)}, Error: {distance_error:.3f} m")

        # ICP 추정 위치를 /initialpose 토픽으로 퍼블리시 (RViz 초기 위치 자동 설정용)
        self.publish_initialpose(est_position)

    def convert_ros_to_o3d(self, ros_cloud: PointCloud2):
        points = []
        for p in pc2.read_points(ros_cloud, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.array(points))
        return pc

    def publish_initialpose(self, position):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = 0.0
        # orientation은 0,0,0,1 단위 쿼터니언으로 고정 (방향 추정 미반영)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        # covariance는 0으로 초기화
        msg.pose.covariance = [0.0]*36

        self.initialpose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ICPWithGTNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
