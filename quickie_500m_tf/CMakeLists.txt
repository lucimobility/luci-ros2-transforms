cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(quickie_500m_tf_node src/tf_broadcaster.cpp)
ament_target_dependencies(quickie_500m_tf_node "tf2_ros" "rclcpp" "geometry_msgs")

install(TARGETS
quickie_500m_tf_node
    DESTINATION lib/${PROJECT_NAME}
)
