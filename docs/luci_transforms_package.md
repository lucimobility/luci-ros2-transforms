# luci_transforms

## summary

This package handles the transformation of raw sensor points that are all referenced at sensor origin to a common chair center origin.

_**Note: All the sensors from LUCI streams are conveniently  already set relative to the chair center so the transforms below are an identity matrix that is only here to provide other tools such as nav\_2 and rviz2 with knowledge of the frames used**_

| Package | Node |
|---------|------|
| luci_transforms | permobil_m3_tf_node |
| luci_transforms | quickie_500m_tf_node |

| Topics | Subscription / Publish | Message Type | Description |
|--------|------------------------|--------------|-------------|
| luci/camera_transform | publish | geometry_msgs::msg::TransformStamped | Transformation from front camera pointcloud to chair center <br/><br/> (base_link = chair center, base_camera = camera stream) |
| luci/ultrasonic_transform | publish | geometry_msgs::msg::TransformStamped | Transformation from ultransonic pointcloud to chair center <br/><br/> (base_link = chair center, base_ultrasonic = ultrasonic stream) |
| luci/radar_transform | publish | geometry_msgs::msg::TransformStamped | Transformation from radar pointcloud to chair center <br/><br/> (base_link = chair center, base_radar = radar stream) |

**Note chair center is defined as the center of the drive wheels and is the 0 point for all sensor streams from LUCI. See image below for reference.**

![luci chair center image](luci-chair-center.png)
