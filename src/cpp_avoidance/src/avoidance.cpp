#include "avoidance.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>


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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);


    // Downsample the cloud by selecting every 1000th point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < cloud->points.size(); i += 100) {
        cloud_downsampled->points.push_back(cloud->points[i]);
    }

    ROS_INFO("Amount of Points %lu", cloud_downsampled->points.size());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 30); // Keep points at a distance of 0 to "range" meters
    pass.filter(*cloud_downsampled);
    PointCloudPtr cloud_downranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downranged = cloud_downsampled;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link", ros::Time::now(), ros::Duration(1));
        pcl_ros::transformPointCloud(*cloud_downranged, *cloud_transformed, transform_stamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    // Perform any necessary filtering on the cloud
    cloud_filtered = groupPoints(cloud_transformed);

    clusters_ = cloud_filtered;
    publishClusters(cloud_filtered);
}

PointCloudPtr Avoidance::groupPoints(const PointCloudPtr& cloud) {
    ros::Time start_time = ros::Time::now();

    // Create KdTree for DBSCAN
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Perform DBSCAN
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.7); // 70cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Compute centroids
    PointCloudPtr centroids(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& indices : cluster_indices) {
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& index : indices.indices) {
            centroid += Eigen::Vector3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z);
        }
        centroid /= indices.indices.size();
        centroids->points.push_back(pcl::PointXYZ(centroid(0), centroid(1), centroid(2)));
    }
    
    ros::Time end_time = ros::Time::now();
    ros::Duration elapsed_time = end_time - start_time;

    ROS_INFO("Time taken by groupPoints: %f seconds", elapsed_time.toSec());
    return centroids;
}

// void Avoidance::potentialFieldsAvoidance()


void Avoidance::publishClusters(const PointCloudPtr& clusters) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < clusters->size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (*clusters)[i].x;
        marker.pose.position.y = (*clusters)[i].y;
        marker.pose.position.z = (*clusters)[i].z;
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
    ROS_INFO("Markers Published!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance_node_cpp");
    ros::NodeHandle nh;

    Avoidance avoidance(nh);
    while (ros::ok())
        ros::spin();

    return 0;
}