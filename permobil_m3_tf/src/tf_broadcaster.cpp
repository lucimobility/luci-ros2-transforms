// Copyright 2025 LUCI Mobility, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_tf_publisher");

    rclcpp::Rate loop_rate(20);

    auto camera_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto radar_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto ultrasonic_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    while (rclcpp::ok())
    {
        rclcpp::Time now = node->get_clock()->now();

        // Camera base frame
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = now;
        camera_tf.header.frame_id = "base_link";
        camera_tf.child_frame_id = "base_camera";
        camera_tf.transform.translation.x = 0;
        camera_tf.transform.translation.y = 0;
        camera_tf.transform.translation.z = 0;
        tf2::Quaternion quat_camera;
        // TODO feed in actual degrees based on camera angles 0, 0, -0.707, 0.707
        quat_camera.setRPY(0, 0, 0);
        camera_tf.transform.rotation.x = quat_camera.x();
        camera_tf.transform.rotation.y = quat_camera.y();
        camera_tf.transform.rotation.z = quat_camera.z();
        camera_tf.transform.rotation.w = quat_camera.w();

        // Radar base frame
        geometry_msgs::msg::TransformStamped radar_tf;
        radar_tf.header.stamp = now;
        radar_tf.header.frame_id = "base_link";
        radar_tf.child_frame_id = "base_radar";
        radar_tf.transform.translation.x = 0;
        radar_tf.transform.translation.y = 0;
        radar_tf.transform.translation.z = 0;
        tf2::Quaternion quat_radar;
        quat_radar.setRPY(0, 0, 0);
        radar_tf.transform.rotation.x = quat_radar.x();
        radar_tf.transform.rotation.y = quat_radar.y();
        radar_tf.transform.rotation.z = quat_radar.z();
        radar_tf.transform.rotation.w = quat_radar.w();

        // Ultrasonic base frame
        geometry_msgs::msg::TransformStamped ultrasonic_tf;
        ultrasonic_tf.header.stamp = now;
        ultrasonic_tf.header.frame_id = "base_link";
        ultrasonic_tf.child_frame_id = "base_ultrasonic";
        ultrasonic_tf.transform.translation.x = 0;
        ultrasonic_tf.transform.translation.y = 0;
        ultrasonic_tf.transform.translation.z = 0;
        tf2::Quaternion quat_ultrasonic;
        quat_ultrasonic.setRPY(0, 0, 0);
        ultrasonic_tf.transform.rotation.x = quat_ultrasonic.x();
        ultrasonic_tf.transform.rotation.y = quat_ultrasonic.y();
        ultrasonic_tf.transform.rotation.z = quat_ultrasonic.z();
        ultrasonic_tf.transform.rotation.w = quat_ultrasonic.w();

        camera_broadcaster->sendTransform(camera_tf);
        radar_broadcaster->sendTransform(radar_tf);
        ultrasonic_broadcaster->sendTransform(ultrasonic_tf);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}
