cmake_minimum_required(VERSION 3.5)
message(STATUS "Processing CMakeLists.txt for traffic_light_system")
project(traffic_light_system)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install the Python package
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
