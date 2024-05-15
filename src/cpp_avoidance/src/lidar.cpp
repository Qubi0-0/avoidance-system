#include "lidar.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>
#include <string.h>

AvoidanceLidar::AvoidanceLidar(const ros::NodeHandle& nh)
    : nh_(nh), octree_(0.1), tf_listener_(tf_buffer_) {
    tf_buffer_.setUsingDedicatedThread(true);
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1, &AvoidanceLidar::scanCallback, this);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/output/octomap_topic", 1);
} 

void AvoidanceLidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    sensor_msgs::PointCloud2 cloud_msg;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan_msg, cloud_msg);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;


    try {
        if (tf_buffer_.canTransform(FROM_TF, TO_TF, ros::Time(0), ros::Duration(1.0))) {
            transform_stamped = tf_buffer_.lookupTransform(FROM_TF, TO_TF, ros::Time(0));
            pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
        } else {
            ROS_WARN("Transform from 'odom' to 'camera_link' not available");
            return;
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    octomap::Pointcloud octomap_cloud;
    for (const auto& point : cloud_transformed->points) {
        octomap_cloud.push_back(point.x, point.y, point.z);
    }

    octree_.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0));
    octree_.updateInnerOccupancy();
    octree_.toMaxLikelihood();

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = "odom";  
    octomap_msgs::binaryMapToMsg(octree_, octomap_msg);

    octomap_pub_.publish(octomap_msg);
    ROS_INFO("Octomap Published");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    AvoidanceLidar avoidance_octomap(nh);
    ros::Rate loop_rate(10);  

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();  
    }

    return 0;
}
