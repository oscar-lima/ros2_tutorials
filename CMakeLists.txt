cmake_minimum_required(VERSION 3.5)
project(ros2_tutorials)

# compile with C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
    ros/include
)

add_executable(talker ros/src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener ros/src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(
  TARGETS talker listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
