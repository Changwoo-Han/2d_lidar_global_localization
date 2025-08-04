cmake_policy(SET CMP0074 NEW)
cmake_minimum_required(VERSION 3.8)
project(ndt_localizer)

## 컴파일러 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(gazebo_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

## 실행 파일 생성
add_executable(ndt_localizer_node
  src/ndt_localizer_node.cpp
)

ament_target_dependencies(ndt_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
  gazebo_msgs
)

target_link_libraries(ndt_localizer_node
  ${PCL_LIBRARIES}
)

## 설치 설정
install(TARGETS
  ndt_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

