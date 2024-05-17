#include "octomap.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>

geometry_msgs::Point AvoidanceOctomap::TARGET_POINT = geometry_msgs::Point();

AvoidanceOctomap::AvoidanceOctomap(const ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
    tf_buffer_.setUsingDedicatedThread(true);

    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &AvoidanceOctomap::positionCallback, this);
    octree_sub_ = nh_.subscribe<octomap_msgs::Octomap>("octomap_binary", 1, &AvoidanceOctomap::octreeCallback, this);

    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("drone_tracking/goal", 1);
    
    TARGET_POINT.x = 0;
    TARGET_POINT.y = 180;
    TARGET_POINT.z = 10;
}

void AvoidanceOctomap::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_position_.x = msg->pose.position.x;
    drone_position_.y = msg->pose.position.y;
    drone_position_.z = msg->pose.position.z;
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

    if (!has_reached_target(drone_position_, point_msg.point)) {
        goal_pub_.publish(point_msg);
    }   
}

bool AvoidanceOctomap::has_reached_target(const geometry_msgs::Point& current_position, const geometry_msgs::Point& target_point) {
    double distance = sqrt(pow(target_point.x - current_position.x, 2) +
                           pow(target_point.y - current_position.y, 2) +
                           pow(target_point.z - current_position.z, 2));
    return distance < 0.1;
}

geometry_msgs::Point AvoidanceOctomap::get_nearest_point_to_target(octomap::OcTree* octree, const geometry_msgs::Point& target_point) {
    const double DIST = 10.0; 

    // Calculate vector from drone to target
    geometry_msgs::Point vector_to_target;
    vector_to_target.x = target_point.x - drone_position_.x;
    vector_to_target.y = target_point.y - drone_position_.y;
    vector_to_target.z = target_point.z - drone_position_.z;

    // Normalize the vector
    double magnitude = sqrt(pow(vector_to_target.x, 2) +
                            pow(vector_to_target.y, 2) +
                            pow(vector_to_target.z, 2));
    vector_to_target.x /= magnitude;
    vector_to_target.y /= magnitude;
    vector_to_target.z /= magnitude;

    // Scale the vector by DIST
    vector_to_target.x *= DIST;
    vector_to_target.y *= DIST;
    vector_to_target.z *= DIST;

    // Calculate the nearest point
    geometry_msgs::Point nearest_point;
    nearest_point.x = drone_position_.x + vector_to_target.x;
    nearest_point.y = drone_position_.y + vector_to_target.y;
    nearest_point.z = drone_position_.z + vector_to_target.z;

    // Check if the point is in a free space in the octree
    octomap::OcTreeNode* node = octree->search(nearest_point.x, nearest_point.y, nearest_point.z);
    if (node != NULL && octree->isNodeOccupied(node)) {
        // The point is in an occupied space, return a default point
        return geometry_msgs::Point();
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
