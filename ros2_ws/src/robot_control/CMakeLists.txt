cmake_minimum_required(VERSION 3.5)
project(robot_control)

# ROS 2 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Python 스크립트 설치
install(PROGRAMS
  slam_node.py
  move_base_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament 패키지 등록
ament_package()