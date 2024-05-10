#include "octomap.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>

AvoidanceOctomap::AvoidanceOctomap(const ros::NodeHandle& nh) : nh_(nh), octree_(0.1) {
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &AvoidanceOctomap::cloudCallback, this);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/output/octomap_topic", 1);
}

void AvoidanceOctomap::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 29); // Keep points at a distance of 0 to "range" meters
    pass.filter(*cloud);
    PointCloudPtr cloud_downranged(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downranged = cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link", ros::Time::now(), ros::Duration(1));
        pcl_ros::transformPointCloud(*cloud_downranged, *cloud_transformed, transform_stamped.transform);
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
    octomap_msgs::binaryMapToMsg(octree_, octomap_msg);
    octomap_pub_.publish(octomap_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance_octomap_node");
    ros::NodeHandle nh;
    AvoidanceOctomap avoidance_octomap(nh);
    ros::spin();
    return 0;
}
