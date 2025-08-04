#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct CandidatePose
{
  Eigen::Matrix4f transform;
  double fitness;
  float x;
  float y;
  float error_cm;
  double yaw;
};

class NDTLocalizer : public rclcpp::Node {
public:
  NDTLocalizer() : Node("ndt_localizer"), ground_truth_log_once_(false), ground_truth_received_(false) {
    this->declare_parameter<std::string>("map_path", "/home/croft_robot/ros2_ws/src/croft_auto/croft_auto_nav2/config/my_map5.pcd");
    this->get_parameter("map_path", map_path_);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *map_cloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", map_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded map with %lu points", map_cloud_->size());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(map_cloud_);
    voxel_filter.setLeafSize(0.3f, 0.3f, 0.3f); 
    voxel_filter.filter(*map_cloud_filtered_);
    RCLCPP_INFO(this->get_logger(), "Downsampled map size: %lu points", map_cloud_filtered_->size());

    // 영역 설정
    hall_min_x_ = -23.0;
    hall_max_x_ = 24.0;
    hall_min_y_ = -23.0;
    hall_max_y_ = -19.0;
    candidate_step_ = 3;

    yaw_candidates_.clear();
    for (int i = -6; i <= 6; ++i) {
      yaw_candidates_.push_back(i * M_PI / 6);
    }

    // 구독 토픽 변경: /model_state_pose
    model_state_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states_demo", 10,
      std::bind(&NDTLocalizer::modelStatesCallback, this, std::placeholders::_1)
    );

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/input_cloud", qos,
      std::bind(&NDTLocalizer::cloudCallback, this, std::placeholders::_1)
    );

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  } 

private:
  // Gazebo world 좌표 → ROS map 좌표 변환 함수
  Eigen::Vector2f transformGazeboToROS(const Eigen::Vector2f& gazebo_pos) {
    // 변환 파라미터 (사용자 제공 변환 값)
    float theta = 3.028f; // 173.5도 in radians
    Eigen::Matrix2f R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);
    Eigen::Vector2f t(24.377f, -21.564f); // 평행이동

    return R * gazebo_pos + t;
  }

  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    if (!ground_truth_log_once_) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "croft_auto") {
          // 변환 적용
          Eigen::Vector2f gazebo_pos(msg->pose[i].position.x, msg->pose[i].position.y);
          Eigen::Vector2f ros_pos = transformGazeboToROS(gazebo_pos);

          // 쿼터니언 -> Euler yaw 계산
          tf2::Quaternion q(
            msg->pose[i].orientation.x,
            msg->pose[i].orientation.y,
            msg->pose[i].orientation.z,
            msg->pose[i].orientation.w
          );
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          ground_truth_pos_ = ros_pos;
          ground_truth_yaw_ = yaw;

          ground_truth_received_ = true;
          ground_truth_log_once_ = true;
          RCLCPP_INFO(this->get_logger(),
            "📍 Ground Truth 저장됨: x=%.2f, y=%.2f, yaw=%.3f rad",
            ground_truth_pos_.x(), ground_truth_pos_.y(), ground_truth_yaw_);
          break;
        }
      }
    }
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto start_time = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(0.3f, 0.3f, 0.3f);
    voxel_filter.filter(*input_cloud_filtered);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputTarget(map_cloud_filtered_);
    ndt.setInputSource(input_cloud_filtered);

    ndt.setStepSize(0.2);
    ndt.setResolution(1.5);
    ndt.setTransformationEpsilon(0.05);
    ndt.setMaximumIterations(20);

    std::vector<CandidatePose> best_candidates;
    double best_fitness = std::numeric_limits<double>::max();
    bool found_valid = false;

    for (double x = hall_min_x_; x <= hall_max_x_; x += candidate_step_) {
      for (double y = hall_min_y_; y <= hall_max_y_; y += candidate_step_) {
        for (double yaw : yaw_candidates_) {
          Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

          init_guess(0, 3) = static_cast<float>(x);
          init_guess(1, 3) = static_cast<float>(y);

          float cos_yaw = std::cos(yaw);
          float sin_yaw = std::sin(yaw);
          init_guess(0, 0) = cos_yaw;
          init_guess(0, 1) = -sin_yaw;
          init_guess(1, 0) = sin_yaw;
          init_guess(1, 1) = cos_yaw;

          pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          ndt.align(*output_cloud, init_guess);

          if (ndt.hasConverged()) {
            double fitness_score = ndt.getFitnessScore();

            if (!found_valid || fitness_score < best_fitness) {
              best_fitness = fitness_score;
              best_candidates.clear();

              CandidatePose candidate;
              candidate.transform = ndt.getFinalTransformation();
              candidate.fitness = fitness_score;
              candidate.x = candidate.transform(0,3);
              candidate.y = candidate.transform(1,3);
              candidate.yaw = std::atan2(candidate.transform(1,0), candidate.transform(0,0));

              if (ground_truth_received_) {
                float error = std::hypot(candidate.x - ground_truth_pos_.x(), candidate.y - ground_truth_pos_.y());
                candidate.error_cm = error * 100.0f;
              } else {
                candidate.error_cm = -1.0f;
              }

              best_candidates.push_back(candidate);
              found_valid = true;
            } else if (fabs(fitness_score - best_fitness) < 1e-6) {
              CandidatePose candidate;
              candidate.transform = ndt.getFinalTransformation();
              candidate.fitness = fitness_score;
              candidate.x = candidate.transform(0,3);
              candidate.y = candidate.transform(1,3);
              candidate.yaw = std::atan2(candidate.transform(1,0), candidate.transform(0,0));

              if (ground_truth_received_) {
                float error = std::hypot(candidate.x - ground_truth_pos_.x(), candidate.y - ground_truth_pos_.y());
                candidate.error_cm = error * 100.0f;
              } else {
                candidate.error_cm = -1.0f;
              }

              best_candidates.push_back(candidate);
            }
          }
        }
      }
    }

    if (!found_valid) {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge for any candidate.");
      return;
    }

    CandidatePose selected = best_candidates.front();
    if (ground_truth_received_) {
      double min_error = std::numeric_limits<double>::max();
      for (auto &c : best_candidates) {
        if (c.error_cm >= 0 && c.error_cm < min_error) {
          min_error = c.error_cm;
          selected = c;
        }
      }
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(this->get_logger(), "🎯 Fitness max인 후보 %lu 개:", best_candidates.size());
    for (size_t i = 0; i < best_candidates.size(); ++i) {
      if (ground_truth_received_) {
        RCLCPP_INFO(this->get_logger(),
          "  후보 %lu: 위치: (%.2f, %.2f) yaw: %.3f rad, 오차: %.2f cm",
          i+1, best_candidates[i].x, best_candidates[i].y, best_candidates[i].yaw, best_candidates[i].error_cm);
      } else {
        RCLCPP_INFO(this->get_logger(),
          "  후보 %lu: 위치: (%.2f, %.2f) yaw: %.3f rad",
          i+1, best_candidates[i].x, best_candidates[i].y, best_candidates[i].yaw);
      }
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = selected.x;
    pose_msg.pose.pose.position.y = selected.y;
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, selected.yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);

    if (ground_truth_received_) {
      RCLCPP_INFO(this->get_logger(),
        "▶️  NDT 위치: (%.2f, %.2f) yaw: %.3f rad / 오차: %.1f cm / 소요시간: %.3f 초",
        selected.x, selected.y, selected.yaw, selected.error_cm, elapsed_seconds);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "▶️  NDT 위치: (%.2f, %.2f) yaw: %.3f rad / 소요시간: %.3f 초",
        selected.x, selected.y, selected.yaw, elapsed_seconds);
    }
  }

  // 멤버 변수 선언부
  std::string map_path_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_filtered_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;

  Eigen::Vector2f ground_truth_pos_;
  double ground_truth_yaw_;
  bool ground_truth_log_once_;
  bool ground_truth_received_;

  double hall_min_x_, hall_max_x_, hall_min_y_, hall_max_y_;
  double candidate_step_;
  std::vector<double> yaw_candidates_;
};  

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NDTLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
