#include "avoidance.h"
#include "pcl_ros/transforms.h"


using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PosePointRPY::PosePointRPY(double x, double y, double z, double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {
    pos.pose.position.x = x;
    pos.pose.position.y = y;
    pos.pose.position.z = z;
    tf::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(quaternion, pos.pose.orientation);
}

Avoidance::Avoidance(const ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &Avoidance::positionCallback, this);
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &Avoidance::cloudCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    fixed_positions_.push_back(PosePointRPY(0, 0, FLIGHT_ALT, 0, 0, M_PI));
    // Define other fixed positions as needed

    flight_status_ = Status::Takeoff;
    yaw_angle_ = M_PI;
    last_published_ = ros::Time::now();
}

void Avoidance::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pose_ = *msg;
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    local_yaw_ = yaw;
    drone_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void Avoidance::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    PointCloudPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    // Perform any necessary filtering on the cloud
    *cloud_filtered = cloud;

    // Downsample the cloud by selecting every 1000th point
    PointCloudPtr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < cloud_filtered->points.size(); i += 1000) {
        cloud_downsampled->points.push_back(cloud_filtered->points[i]);
    }

    PointCloudPtr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link", ros::Time::now(), ros::Duration(1));
        pcl_ros::transformPointCloud(*cloud_downsampled, *cloud_transformed, transform_stamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    // Group the points
    // Implement point grouping logic using DBSCAN or any other method

    // Publish the clstered points
    std::vector<Eigen::Vector3d> clusters;
    for (const auto& point : cloud_transformed->points) {
        clusters.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    // Populate 'clusters' with clustered points
    publishClusters(clusters);
}


void Avoidance::publishClusters(const std::vector<Eigen::Vector3d>& clusters) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < clusters.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = clusters[i](0);
        marker.pose.position.y = clusters[i](1);
        marker.pose.position.z = clusters[i](2);
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.id = i;
        marker_array.markers.push_back(marker);
    }
    marker_pub_.publish(marker_array);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance_node_cpp");
    ros::NodeHandle nh;

    Avoidance avoidance(nh);
    while (ros::ok())
        ros::spin();

    return 0;
}