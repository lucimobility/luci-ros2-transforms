/***
 * Note: This file is NOT needed if using an accurate URDF as the transforms are handled by the
 * robot_description node The transforms here are also mildly misleading as they suggest that the
 * sensors are inline with the chairs base link and just need to be rotated 90 degrees on the chairs
 * z axis. This is because the transforms in this file are for the gRPC streams which on LUCI have
 * already been transformed to be in the chairs base_link frame. The 90 rotation on z in this file
 * is because LUCI considers forward as the Y axis and ROS has forward as being the X axis. So
 * points from gRPC that are in front of the chair will have a Y distance value and to get them
 * compatible with ROS we rotate them to have a X axis value.
 *
 */
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

/**
 * @brief The static frame publisher used in conjunction with the gRPC interface node to transform
 * the poinclouds to a ROS native axis for base_link.
 *
 */
class StaticFramePublisher : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Static Frame Publisher object
     * Node name = "luci_tf_publisher"
     *
     */
    explicit StaticFramePublisher() : Node("luci_tf_publisher")
    {
        /// Static transform broadcasters
        camera_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        radar_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        ultrasonic_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        footprint_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        /// Actually makes transformations and sends them
        this->makeTransforms();
    }

  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> camera_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> radar_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> ultrasonic_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> footprint_broadcaster;

    /**
     * @brief Makes all transforms available to the ROS2 network
     *
     */
    void makeTransforms()
    {
        // Get current time transforms are being published (time is ROS synced)
        rclcpp::Time now = this->get_clock()->now();

        // Camera stream frame
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = now;
        camera_tf.header.frame_id = "base_link";
        camera_tf.child_frame_id = "base_camera";
        camera_tf.transform.translation.x = 0;
        camera_tf.transform.translation.y = 0;
        camera_tf.transform.translation.z = 0;
        tf2::Quaternion quat_camera;
        quat_camera.setRPY(
            0, 0, -1.5708); // Rotate from base_link to base_camera is -90 degrees in z axis
        camera_tf.transform.rotation.x = quat_camera.x();
        camera_tf.transform.rotation.y = quat_camera.y();
        camera_tf.transform.rotation.z = quat_camera.z();
        camera_tf.transform.rotation.w = quat_camera.w();

        // Radar stream frame
        geometry_msgs::msg::TransformStamped radar_tf;
        radar_tf.header.stamp = now;
        radar_tf.header.frame_id = "base_link";
        radar_tf.child_frame_id = "base_radar";
        radar_tf.transform.translation.x = 0;
        radar_tf.transform.translation.y = 0;
        radar_tf.transform.translation.z = 0;
        tf2::Quaternion quat_radar;
        // Rotate from base_link to base_radar is -90 degrees in z axis
        quat_radar.setRPY(0, 0, -1.5708);
        radar_tf.transform.rotation.x = quat_radar.x();
        radar_tf.transform.rotation.y = quat_radar.y();
        radar_tf.transform.rotation.z = quat_radar.z();
        radar_tf.transform.rotation.w = quat_radar.w();

        // Ultrasonic stream frame
        geometry_msgs::msg::TransformStamped ultrasonic_tf;
        ultrasonic_tf.header.stamp = now;
        ultrasonic_tf.header.frame_id = "base_link";
        ultrasonic_tf.child_frame_id = "base_ultrasonic";
        ultrasonic_tf.transform.translation.x = 0;
        ultrasonic_tf.transform.translation.y = 0;
        ultrasonic_tf.transform.translation.z = 0;
        tf2::Quaternion quat_ultrasonic;
        // Rotate from base_link to base_ultrasonic is -90 degrees in z axis
        quat_ultrasonic.setRPY(0, 0, -1.5708);
        ultrasonic_tf.transform.rotation.x = quat_ultrasonic.x();
        ultrasonic_tf.transform.rotation.y = quat_ultrasonic.y();
        ultrasonic_tf.transform.rotation.z = quat_ultrasonic.z();
        ultrasonic_tf.transform.rotation.w = quat_ultrasonic.w();

        // Base footprint
        geometry_msgs::msg::TransformStamped footprint_tf;
        footprint_tf.header.stamp = now;
        footprint_tf.header.frame_id = "base_link";
        footprint_tf.child_frame_id = "base_footprint";
        footprint_tf.transform.translation.x = 0;
        footprint_tf.transform.translation.y = 0;
        // Translation from base_link to base_footprint is -0.17 meters on z axis (how high is
        // center of chair from the ground)
        footprint_tf.transform.translation.z = -0.17;
        tf2::Quaternion quat_footprint;
        quat_footprint.setRPY(0, 0, 0);
        footprint_tf.transform.rotation.x = quat_footprint.x();
        footprint_tf.transform.rotation.y = quat_footprint.y();
        footprint_tf.transform.rotation.z = quat_footprint.z();
        footprint_tf.transform.rotation.w = quat_footprint.w();

        this->camera_broadcaster->sendTransform(camera_tf);
        this->radar_broadcaster->sendTransform(radar_tf);
        this->ultrasonic_broadcaster->sendTransform(ultrasonic_tf);
        this->footprint_broadcaster->sendTransform(footprint_tf);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
