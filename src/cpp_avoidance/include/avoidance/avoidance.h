#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string.h>


#define SOURCE_FRAME "camera_link"
#define TARGET_FRAME "odom"
#define FLIGHT_ALT 10 // altitude for drone flight
#define M_PI 3.14159265358979323846
#define POS_TRESHOLD 0.1
#define HEIGHT_TRESHOLD 1
// POTENTIAL FORCE PARAMS
#define K_ATT 0.06  // Attractive force constant
#define K_REP 2.5    // Repulsive force constant
#define FORCE_TREHSHOLD 2.0
// DBSCAN PARAMS
#define CLUSTER_TOLERANCE 0.5 // In m (max distance from points in one cluster)
#define MIN_CLUSTER_SIZE 20
#define MAX_CLUSTER_SIZE 3000

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class PosePointRPY {
public:
    PosePointRPY(double x, double y, double z, double roll, double pitch, double yaw);

    geometry_msgs::PoseStamped pos;
    double roll, pitch, yaw;
};

class Avoidance {
public:
    Avoidance(const ros::NodeHandle& nh);

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void publishClusters(const PointCloudPtr& clusters);
    PointCloudPtr groupPoints(const PointCloudPtr& cloud);
    Eigen::Vector3d computeRepulsiveForce();
    Eigen::Vector3d computeAttractiveForce();
    double computeHeightForce();
    void potentialFieldsAvoidance();

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    Eigen::Vector3d drone_position_;
    geometry_msgs::PoseStamped local_pose_;
    double local_yaw_;
    double yaw_angle_;
    Eigen::Vector3d target_position_;
    Eigen::MatrixXd obstacles_;
    Eigen::MatrixXd pre_transformed_;
    ros::Time last_req_;
    ros::Time last_published_;
    PointCloudPtr clusters_;
public:
    // Timers
    ros::Time mean_cloud_time;
    int cloud_count = 0;
    ros::Time mean_DBSCAN_time;
    int DBSCAN_count = 0;
    ros::Time mean_flight_time;
    int flight_count = 0;

};

#endif // AVOIDANCE_H
