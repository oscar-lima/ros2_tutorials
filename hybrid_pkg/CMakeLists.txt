cmake_minimum_required(VERSION 3.5)
project(hybrid_pkg)

# compile with C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(talker_cpp src/talker.cpp)
ament_target_dependencies(talker_cpp rclcpp std_msgs)

add_executable(listener_cpp src/listener.cpp)
ament_target_dependencies(listener_cpp rclcpp std_msgs)

install(
  TARGETS talker_cpp listener_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY nodes
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
