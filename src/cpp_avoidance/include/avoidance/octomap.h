#ifndef AVOIDANCE_OCTOMAP_H
#define AVOIDANCE_OCTOMAP_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class AvoidanceOctomap {
public:
    AvoidanceOctomap(const ros::NodeHandle& nh);

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void generateOctomap();

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher octomap_pub_;
    ros::Publisher grid_pub_;
    octomap::OcTree octree_;
    tf2_ros::Buffer tf_buffer_;
};

#endif // AVOIDANCE_OCTOMAP_H
