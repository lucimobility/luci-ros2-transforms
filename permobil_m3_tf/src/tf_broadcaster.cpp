#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle node_handle;
    ros::Rate rate(20);

    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, -0.707, 0.707), tf::Vector3(0, 0, 0)), ros::Time::now(), "base_link", "base_camera"));
        rate.sleep();
    }
}