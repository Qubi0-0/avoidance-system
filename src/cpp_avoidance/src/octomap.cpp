#include "octomap.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>

AvoidanceOctomap::AvoidanceOctomap(const ros::NodeHandle& nh) : nh_(nh), octree_(0.1) {
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &AvoidanceOctomap::cloudCallback, this);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/output/octomap_topic", 1);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/output/occupancy_grid",5);
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

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    // geometry_msgs::TransformStamped transform_stamped;
    // try {
    //     transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link", ros::Time::now(), ros::Duration(1));
    //     pcl_ros::transformPointCloud(*cloud_downranged, *cloud_transformed, transform_stamped.transform);
    // } catch (tf2::TransformException& ex) {
    //     ROS_WARN("%s", ex.what());
    //     return;
    // }

    octomap::Pointcloud octomap_cloud;
    for (const auto& point : cloud_downranged->points) {
        octomap_cloud.push_back(point.x, point.y, point.z);
    }

        octree_.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0));

        octree_.updateInnerOccupancy();
        octree_.toMaxLikelihood();
        

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = "camera_link";  
    octomap_msgs::binaryMapToMsg(octree_, octomap_msg);


    octomap_pub_.publish(octomap_msg);

        
        nav_msgs::OccupancyGrid grid_msg;
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();

        for (octomap::OcTree::leaf_iterator it = octree_.begin_leafs(), end = octree_.end_leafs(); it != end; ++it) {
            min_x = std::min(min_x, it.getX());
            min_y = std::min(min_y, it.getY());
        }

        grid_msg.info.origin.position.x = min_x;
        grid_msg.info.origin.position.y = min_y;
        // Set the size of the grid
        grid_msg.info.width = 100;  // Change this to your desired width
        grid_msg.info.height = 100;  // Change this to your desired height
        grid_msg.data = std::vector<int8_t>(grid_msg.info.width * grid_msg.info.height, -1);
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "camera_link";  
        grid_msg.info.resolution = 1.1; 

      

        for (octomap::OcTree::leaf_iterator it = octree_.begin_leafs(), end = octree_.end_leafs(); it != end; ++it) {
            if (octree_.isNodeOccupied(*it)) {
                double x = it.getX();
                double y = it.getY();
                int i = (x - grid_msg.info.origin.position.x) / grid_msg.info.resolution;
                int j = (y - grid_msg.info.origin.position.y) / grid_msg.info.resolution;
                int index = i + j * grid_msg.info.width;
                if (index >= 0 && index < grid_msg.data.size()) {
                    grid_msg.data[index] = 100;
                } else {
                    ROS_WARN("Invalid index: %d (grid size: %zu)", index, grid_msg.data.size());
                }
            }
        }

        grid_pub_.publish(grid_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance_octomap_node");
    ros::NodeHandle nh;
    AvoidanceOctomap avoidance_octomap(nh);
    ros::Rate loop_rate(10);  

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();  
    }

    return 0;
}