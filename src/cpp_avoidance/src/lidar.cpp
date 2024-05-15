#include "lidar.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>

AvoidanceLidar::AvoidanceLidar(const ros::NodeHandle& nh)
    : nh_(nh), octree_(0.1), tf_listener_(tf_buffer_) {
    tf_buffer_.setUsingDedicatedThread(true);
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &AvoidanceLidar::cloudCallback, this);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/output/octomap_topic", 1);
}

void AvoidanceLidar::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Downsample the cloud by selecting every Nth point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < cloud->points.size(); i += 100) {
        cloud_downsampled->points.push_back(cloud->points[i]);
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 30); 
    pass.filter(*cloud_downsampled);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downranged = cloud_downsampled;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;
    try {
        if (tf_buffer_.canTransform("odom", "camera_link", ros::Time(0), ros::Duration(1.0))) {
            transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link", ros::Time(0));
            pcl_ros::transformPointCloud(*cloud_downranged, *cloud_transformed, transform_stamped.transform);
        } else {
            ROS_WARN("Transform from 'odom' to 'camera_link' not available");
            return;
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(cloud_transformed);
    pass2.setFilterFieldName("z");
    pass2.setFilterLimits(3.0, 29); // Keep points at a distance of 0 to "range" meters
    pass2.filter(*cloud_transformed);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_upranged = cloud_transformed;


    octomap::Pointcloud octomap_cloud;
    for (const auto& point : cloud_upranged->points) {
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
