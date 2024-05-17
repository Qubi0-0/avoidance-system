#include "octomap.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>

geometry_msgs::Point AvoidanceOctomap::TARGET_POINT = geometry_msgs::Point();

AvoidanceOctomap::AvoidanceOctomap(const ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
    tf_buffer_.setUsingDedicatedThread(true);
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/iris/camera/depth/points", 1, &AvoidanceOctomap::cloudCallback, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &AvoidanceOctomap::positionCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("drone_tracking/goal", 1);
    octree_sub_ = nh_.subscribe<octomap_msgs::Octomap>("octomap_binary", 1, &AvoidanceOctomap::octreeCallback, this);
    TARGET_POINT.x = 0;
    TARGET_POINT.y = 180;
    TARGET_POINT.z = 10;
}


void AvoidanceOctomap::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_position_ = tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
void AvoidanceOctomap::octreeCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
    octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    geometry_msgs::Point target_point;
    target_point.x = TARGET_POINT.x;  
    target_point.y = TARGET_POINT.y;  
    target_point.z = TARGET_POINT.z;  

    geometry_msgs::Point nearest_point = get_nearest_point_to_target(octree_, target_point);

    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = "odom";
    point_msg.point = nearest_point;
    geometry_msgs::Point drone_position;
    drone_position.x = drone_position_.getX();
    drone_position.y = drone_position_.getY();
    drone_position.z = drone_position_.getZ();
    if (!has_reached_target(drone_position, point_msg.point)) {
        goal_pub_.publish(point_msg);
    }   
}

bool AvoidanceOctomap::has_reached_target(const geometry_msgs::Point& current_position, const geometry_msgs::Point& target_point) {
    double distance = sqrt(pow(target_point.x - current_position.x, 2) +
                           pow(target_point.y - current_position.y, 2) +
                           pow(target_point.z - current_position.z, 2));
    return distance < 0.1;
}

geometry_msgs::Point get_nearest_point_to_target(octomap::OcTree* octree, const geometry_msgs::Point& target_point) {
    double min_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::Point nearest_point;

    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it) {
        geometry_msgs::Point current_point;
        current_point.x = it.getX();
        current_point.y = it.getY();
        current_point.z = it.getZ();

        double distance = sqrt(pow(target_point.x - current_point.x, 2) +
                               pow(target_point.y - current_point.y, 2) +
                               pow(target_point.z - current_point.z, 2));

        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = current_point;
        }
    }

    return nearest_point;
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
