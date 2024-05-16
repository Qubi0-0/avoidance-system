#include "octomap.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>

geometry_msgs::Point AvoidanceOctomap::TARGET_POINT = geometry_msgs::Point();

AvoidanceOctomap::AvoidanceOctomap(const ros::NodeHandle& nh)
    : nh_(nh), octree_(0.1), tf_listener_(tf_buffer_) {
    tf_buffer_.setUsingDedicatedThread(true);
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &AvoidanceOctomap::cloudCallback, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &AvoidanceOctomap::positionCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("drone_tracking/goal", 1);
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/output/octomap_topic", 1);
    TARGET_POINT.x = 0;
    TARGET_POINT.y = 180;
    TARGET_POINT.z = 10;
}


void AvoidanceOctomap::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_position_ = tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void AvoidanceOctomap::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
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
        if (tf_buffer_.canTransform(TARGET_FRAME, SOURCE_FRAME, ros::Time(0), ros::Duration(1.0))) {
            transform_stamped = tf_buffer_.lookupTransform(TARGET_FRAME, SOURCE_FRAME, ros::Time(0));
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
    pass2.setFilterLimits(3.0, 29); 
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

    geometry_msgs::Point target_point;
    target_point.x = TARGET_POINT.x;  
    target_point.y = TARGET_POINT.y;  
    target_point.z = TARGET_POINT.z;  

    geometry_msgs::Point nearest_point = get_nearest_point_to_target(&octree_);

    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = "odom";
    point_msg.point = nearest_point;
    goal_pub_.publish(point_msg);


    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = "odom";  
    octomap_msgs::binaryMapToMsg(octree_, octomap_msg);

    octomap_pub_.publish(octomap_msg);

}

geometry_msgs::Point AvoidanceOctomap::get_nearest_point_to_target(octomap::OcTree* octree) {
    double min_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::Point nearest_point;

    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it) {
        geometry_msgs::Point current_point;
        current_point.x = it.getX();
        current_point.y = it.getY();
        current_point.z = it.getZ();

        double distance = sqrt(pow(TARGET_POINT.x - current_point.x, 2) +
                               pow(TARGET_POINT.y - current_point.y, 2) +
                               pow(TARGET_POINT.z - current_point.z, 2));

        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = current_point;
        }
    }


    geometry_msgs::Point midpoint;
    midpoint.x = nearest_point.x + 0.5 * (drone_position_.x() - nearest_point.x);
    midpoint.y = nearest_point.y + 0.5 * (drone_position_.y() - nearest_point.y);
    midpoint.z = nearest_point.z + 0.5 * (drone_position_.z() - nearest_point.z);
\
    octomap::OcTreeNode* result = octree->search(midpoint.x, midpoint.y, midpoint.z);

    if (result == NULL) {
        // The midpoint is not in the octree, handle this situation
        // For example, set midpoint to nearest_point
        midpoint = nearest_point;
    }

    return midpoint;
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
