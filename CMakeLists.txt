cmake_minimum_required(VERSION 3.5)
project(tiago_trajectory)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(send_trajectory src/send_trajectory.cpp)
ament_target_dependencies(send_trajectory rclcpp trajectory_msgs)

install(TARGETS
  send_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
