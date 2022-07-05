cmake_minimum_required(VERSION 3.8)
project(luci_sensors_tf)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster "tf2_ros" "rclcpp" "geometry_msgs")

install(TARGETS
    tf_broadcaster
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()