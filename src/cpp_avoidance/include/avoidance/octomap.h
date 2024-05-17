#ifndef AVOIDANCE_OCTOMAP_H
#define AVOIDANCE_OCTOMAP_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <string.h>

#define SOURCE_FRAME "camera_link"
#define TARGET_FRAME "odom"


class AvoidanceOctomap {
public:
    static geometry_msgs::Point TARGET_POINT;

    AvoidanceOctomap(const ros::NodeHandle& nh);

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void octreeCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    bool has_reached_target(const geometry_msgs::Point& current_position, const geometry_msgs::Point& target_point);
    geometry_msgs::Point get_nearest_point_to_target(octomap::OcTree* octree, const geometry_msgs::Point& target_point);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber octree_sub_;
    ros::Publisher goal_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::Point drone_position_;
    octomap::OcTree* octree_;
};

#endif // AVOIDANCE_OCTOMAP_H
