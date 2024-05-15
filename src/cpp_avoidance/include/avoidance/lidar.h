#ifndef AVOIDANCE_LIDAR_H
#define AVOIDANCE_LIDAR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>

class AvoidanceLidar {
public:
    AvoidanceLidar(const ros::NodeHandle& nh);

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher octomap_pub_;
    octomap::OcTree octree_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif // AVOIDANCE_LIDAR_H
