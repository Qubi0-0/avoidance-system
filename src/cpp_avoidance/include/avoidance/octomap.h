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
#include <tf2/LinearMath/Vector3.h> // Add this line
#include <visualization_msgs/MarkerArray.h>
#include <string.h>

#define SOURCE_FRAME "camera_link"
#define TARGET_FRAME "odom"


class AvoidanceOctomap {
public:
    static geometry_msgs::Point TARGET_POINT;

    AvoidanceOctomap(const ros::NodeHandle& nh);

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool has_reached_target(const geometry_msgs::Point& current_position, const geometry_msgs::Point& target_point);
    geometry_msgs::Point get_nearest_point_to_target(octomap::OcTree* octree);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher octomap_pub_;
    ros::Publisher goal_pub_;
    octomap::OcTree octree_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2::Vector3 drone_position_;
};

#endif // AVOIDANCE_OCTOMAP_H
