cmake_minimum_required(VERSION 3.8)
project(sixdof_rviz_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(joint_quintic_traj_node src/joint_quintic_traj_node.cpp)
ament_target_dependencies(joint_quintic_traj_node
  rclcpp std_msgs sensor_msgs trajectory_msgs builtin_interfaces
)

install(TARGETS joint_quintic_traj_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
