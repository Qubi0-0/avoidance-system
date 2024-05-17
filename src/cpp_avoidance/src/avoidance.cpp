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
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("potential_twist", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    target_position_ = {0, 180, FLIGHT_ALT};
    
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
    ros::Time start_time = ros::Time::now();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);


    // Downsample the cloud by selecting every Nth point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < cloud->points.size(); i += 1000) {
        cloud_downsampled->points.push_back(cloud->points[i]);
    }

    // ROS_INFO("Amount of Points %lu", cloud_downsampled->points.size());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 29); // Keep points at a distance of 0 to "range" meters
    pass.filter(*cloud_downsampled);
    PointCloudPtr cloud_downranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downranged = cloud_downsampled;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(TARGET_FRAME, SOURCE_FRAME, ros::Time::now(), ros::Duration(1));
        pcl_ros::transformPointCloud(*cloud_downranged, *cloud_transformed, transform_stamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    // Perform any necessary filtering on the cloud
    // cloud_filtered = groupPoints(cloud_transformed);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(cloud_transformed);
    pass2.setFilterFieldName("z");
    pass2.setFilterLimits(3.0, 29); // Keep points at a distance of 0 to "range" meters
    pass2.filter(*cloud_transformed);
    PointCloudPtr cloud_upranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_upranged = cloud_transformed;

    
    ros::Time end_time = ros::Time::now();
    ros::Duration elapsed_time = end_time - start_time;
    mean_cloud_time += elapsed_time;
    cloud_count ++;
    clusters_ = cloud_upranged;
    publishClusters(cloud_upranged);
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
    ec.setClusterTolerance(CLUSTER_TOLERANCE); 
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);
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
    mean_DBSCAN_time += elapsed_time;
    DBSCAN_count ++;
    // ROS_INFO("Time taken by groupPoints: %f seconds", elapsed_time.toSec());
    return centroids;
}

    Eigen::Vector3d Avoidance::computeRepulsiveForce() {
        Eigen::Vector3d repulsive_force(0, 0, 0);
        if (!clusters_) {
            ROS_ERROR("Clusters is null");
            return repulsive_force;
        }
        if (!clusters_->points.empty()) {
            for (const auto& obstacle : clusters_->points) {
                // Compute the distance between the drone and the obstacle
                double distance = (drone_position_ - Eigen::Vector3d(obstacle.x, obstacle.y, obstacle.z)).norm();

                // Compute the direction vector away from the obstacle
                Eigen::Vector3d direction_vector = (drone_position_ - Eigen::Vector3d(obstacle.x, obstacle.y, obstacle.z)) / distance;

                // Compute repulsive force using the inverse square law

                repulsive_force += (K_REP / (distance * distance)) * direction_vector;
            }
        }

        return repulsive_force;
    }

Eigen::Vector3d Avoidance::computeAttractiveForce() {
        Eigen::Vector3d current_pos(local_pose_.pose.position.x,
                                    local_pose_.pose.position.y,
                                    local_pose_.pose.position.z);

        Eigen::Vector3d attractive_force = K_ATT * (target_position_ - current_pos);
        return attractive_force;
    }

double Avoidance::computeHeightForce() {
    double force_factor = 1.0;
    double height_difference = FLIGHT_ALT - local_pose_.pose.position.z;
    double z_force = height_difference * force_factor;

    return z_force;
}

void Avoidance::potentialFieldsAvoidance() {
    geometry_msgs::Twist twist_msg;

    Eigen::Vector3d attractive_force = computeAttractiveForce();
    Eigen::Vector3d repulsive_force = computeRepulsiveForce();

    Eigen::Vector3d total_force = attractive_force + repulsive_force;
    yaw_angle_ = atan2(attractive_force[1], attractive_force[0]);

    double yaw_error = yaw_angle_ - local_yaw_;
    yaw_error = fmod((yaw_error + M_PI), (2 * M_PI)) - M_PI;
    double z_force = computeHeightForce();
    total_force[2] = total_force[2] + z_force;

    for (int i = 0; i < 3; i++) {
        if (total_force[i] > FORCE_TREHSHOLD) {
            total_force[i] = FORCE_TREHSHOLD;
        } else if (total_force[i] < -FORCE_TREHSHOLD) {
            total_force[i] = -FORCE_TREHSHOLD;
        }
    }
    twist_msg.linear.x = total_force[0];
    twist_msg.linear.y = total_force[1];
    twist_msg.linear.z = total_force[2];

    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = yaw_error * 0.3;

    // Log info
    ROS_INFO_STREAM("Attractive: x: " << attractive_force[0] << ", y: " << attractive_force[1] << ", z: " << attractive_force[2]);
    ROS_INFO_STREAM("Repulsive: x: " << repulsive_force[0] << ", y: " << repulsive_force[1] << ", z: " << repulsive_force[2]);
    ROS_INFO_STREAM("Total Force: " << total_force);

    if (ros::Time::now() - last_published_ > ros::Duration(0.2)) {
        vel_pub_.publish(twist_msg);
        last_published_ = ros::Time::now();
    }
}

void Avoidance::publishClusters(const PointCloudPtr& clusters) {
    clusters->points.push_back(pcl::PointXYZ(target_position_[0],target_position_[1],target_position_[2]));
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
        if (i == clusters->size() - 1) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.scale.x = 2.8;
        marker.scale.y = 2.8;
        marker.scale.z = 2.8;     
        }
        else {
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        }
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
    while (ros::ok()) {
        ros::spinOnce();
        avoidance.potentialFieldsAvoidance();
        if (avoidance.cloud_count > 0) {
        auto mean_cloud_time = avoidance.mean_cloud_time.toSec() / avoidance.cloud_count;
        ROS_INFO("Average time taken by CloudCallback: %f seconds", mean_cloud_time);
    }
        if (avoidance.DBSCAN_count > 0) {
        auto mean_db_time = avoidance.mean_DBSCAN_time.toSec() / avoidance.DBSCAN_count;
        ROS_INFO("Average time taken by DBSCAN: %f seconds", mean_db_time);
    }
    }

    return 0;
}