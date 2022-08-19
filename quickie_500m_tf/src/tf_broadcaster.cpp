#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_tf_publisher");

    rclcpp::Rate loop_rate(20);

    auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    while (rclcpp::ok())
    {
        rclcpp::Time now = node->get_clock()->now();

        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = now;
        trans.header.frame_id = "base_link";
        trans.child_frame_id = "base_camera";

        trans.transform.translation.x = 0;
        trans.transform.translation.y = 0;
        trans.transform.translation.z = 0;
        tf2::Quaternion quat;
        // TODO feed in actual degrees based on camera angles 0, 0, -0.707, 0.707
        quat.setRPY(0, 0, -0.707);
        trans.transform.rotation.x = quat.x();
        trans.transform.rotation.y = quat.y();
        trans.transform.rotation.z = quat.z();
        trans.transform.rotation.w = quat.w();

        broadcaster->sendTransform(trans);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}