import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import open3d as o3d
import numpy as np
import struct
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class ICPInitialPoseEstimator(Node):
    def __init__(self):
        super().__init__('icp_initial_pose_estimator')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.declare_parameter('map_path', "/home/croft_robot/ros2_ws/src/croft_auto/croft_auto_nav2/config/my_map5.pcd")
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value

        # 원본 맵 로드
        self.map_cloud = o3d.io.read_point_cloud(self.map_path)
        # 맵 다운샘플링 + 노멀 계산 (법선 필수)
        self.map_cloud_down = self.preprocess_pointcloud(self.map_cloud, voxel_size=0.2) #para1 다운샘플링 해상도

        self.ground_truth_pose = None

        self.subscription = self.create_subscription(PointCloud2, '/input_cloud', self.scan_callback, qos)
        self.create_subscription(ModelStates, '/model_states_demo', self.gazebo_pose_callback, 10)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 탐색 영역 제한 (hall 범위)
        self.hall_min_x = -23
        self.hall_max_x = 24
        self.hall_min_y = -23
        self.hall_max_y = -19

        # Yaw 후보 (±180도, 30도 간격, 13개)
        self.yaw_candidates_ = [i * math.pi / 6 for i in range(-6, 6)]

    def preprocess_pointcloud(self, cloud, voxel_size=0.2): #para2 다운샘플링 해상도
        # 다운샘플링
        cloud_down = cloud.voxel_down_sample(voxel_size)
        # 노멀 계산 (point-to-plane ICP 위해 필수)
        cloud_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        # 이상치 제거 (통계적 필터링)
        cloud_filtered, ind = cloud_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return cloud_filtered

    def gazebo_pose_callback(self, msg):
        if self.ground_truth_pose is not None:
            return
        try:
            index = msg.name.index('croft_auto')
            pos = msg.pose[index].position
            ori = msg.pose[index].orientation
            quat = [ori.x, ori.y, ori.z, ori.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.ground_truth_pose = np.array([pos.x, pos.y])
            self.get_logger().info(f"📍 Ground Truth 저장됨: x={pos.x:.2f}, y={pos.y:.2f}, yaw={yaw:.3f} rad")
        except ValueError:
            self.get_logger().warn("croft_auto를 model_states에서 찾지 못함.")

    def generate_candidate_poses_grid_in_hall(self, step=1): #para3 변수 선언(기본값1로 쓰겠다) 
        return [(x, y) for x in np.arange(self.hall_min_x, self.hall_max_x + step, step)
                       for y in np.arange(self.hall_min_y, self.hall_max_y + step, step)]

    def scan_callback(self, msg):
        start_time = time.time()

        # ROS PointCloud2 -> Open3D
        scan_cloud = self.convert_ros_to_o3d(msg)
        # 입력 스캔 전처리: 다운샘플링 + 노멀 계산 + 이상치 제거
        scan_down = self.preprocess_pointcloud(scan_cloud, voxel_size=0.2) #para4 다운샘플링 해상도

        candidates = self.generate_candidate_poses_grid_in_hall(step=1) #para3 변수 호출(실제로 이값 쓰겠다)
        if not candidates:
            self.get_logger().warn("후보 위치가 없습니다.")
            return

        best_fitness = -1
        best_pos = None
        best_yaw = None
        best_result = None

        for (x, y) in candidates:
            for yaw in self.yaw_candidates_:
                init_transform = np.identity(4)
                init_transform[0, 3] = x
                init_transform[1, 3] = y
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                init_transform[0, 0] = cos_yaw
                init_transform[0, 1] = -sin_yaw
                init_transform[1, 0] = sin_yaw
                init_transform[1, 1] = cos_yaw

                result = o3d.pipelines.registration.registration_icp(
                    scan_down, self.map_cloud_down, 1.0, init_transform, # para6. icp매칭 허용거리
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100) #para7, icp 반복횟수
                )

                if result.fitness == 1.0:
                    best_fitness = result.fitness
                    best_pos = result.transformation[:3, 3]
                    best_yaw = math.atan2(result.transformation[1, 0], result.transformation[0, 0])
                    best_result = result
                    break

                if (result.fitness > best_fitness) or \
                   (result.fitness == best_fitness and (best_result is None or result.inlier_rmse < best_result.inlier_rmse)):
                    best_fitness = result.fitness
                    best_pos = result.transformation[:3, 3]
                    best_yaw = math.atan2(result.transformation[1, 0], result.transformation[0, 0])
                    best_result = result

            if best_fitness == 1.0:
                break

        if best_fitness < 0.2:
            self.get_logger().warn(f"❌ ICP 실패: 최고 fitness={best_fitness:.3f}")
            return

        icp_pos = np.array([best_pos[0], best_pos[1]])
        self.publish_initialpose(icp_pos, best_yaw)

        elapsed_time = time.time() - start_time

        if self.ground_truth_pose is not None:
            error = np.linalg.norm(icp_pos - self.ground_truth_pose)
            error_cm = error * 100.0
            self.get_logger().info(
                f"▶️  ICP 위치: ({icp_pos[0]:.2f}, {icp_pos[1]:.2f}) yaw: {best_yaw:.3f} rad"
                f" / 오차: {error_cm:.1f}cm / 소요시간: {elapsed_time:.2f}초"
            )
        else:
            self.get_logger().info(
                f"▶️  ICP 위치: ({icp_pos[0]:.2f}, {icp_pos[1]:.2f}) yaw: {best_yaw:.3f} rad"
                f" 소요시간: {elapsed_time:.2f}초"
            )

    def publish_initialpose(self, pos, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.initialpose_pub.publish(msg)

    def convert_ros_to_o3d(self, msg):
        # PointCloud2 데이터를 Open3D 포인트 클라우드로 변환
        cloud_data = list(struct.iter_unpack('ffff', msg.data))
        points = np.array([[x, y, z] for x, y, z, _ in cloud_data], dtype=np.float32)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = ICPInitialPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
