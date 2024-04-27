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

#define FLIGHT_ALT 10 // altitude for drone flight
#define M_PI 3.14159265358979323846
#define TARGET_ANGL (1 * (M_PI / 180.0))
#define POS_TRESHOLD 0.1
#define HEIGHT_TRESHOLD 1
#define K_ATT 0.01  // Attractive force constant
#define K_REP 1.0    // Repulsive force constant

// using EigenVec = std::vector<Eigen::Vector3d>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class PosePointRPY {
public:
    PosePointRPY(double x, double y, double z, double roll, double pitch, double yaw);

    geometry_msgs::PoseStamped pos;
    double roll, pitch, yaw;
};

enum Status { Takeoff, FlyMode };

class Avoidance {
public:
    Avoidance(const ros::NodeHandle& nh);

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    // void spin();
    void publishClusters(const PointCloudPtr& clusters);
    // void potentialFieldsAvoidance();
    PointCloudPtr groupPoints(const PointCloudPtr& cloud);

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
    Status flight_status_;
    double yaw_angle_;
    std::vector<PosePointRPY> fixed_positions_;
    Eigen::MatrixXd obstacles_;
    Eigen::MatrixXd pre_transformed_;
    ros::Time last_req_;
    ros::Time last_published_;
    PointCloudPtr clusters_;
};

#endif // AVOIDANCE_H
