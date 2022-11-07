cmake_minimum_required(VERSION 3.8)
project(luci_transforms)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_subdirectory(quickie_500m_tf)
add_subdirectory(permobil_m3_tf)

ament_package()
