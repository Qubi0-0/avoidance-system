#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import numpy as np
import math
from tf import transformations
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

FLIGHT_ALT = 10 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.1
HEIGHT_TRESHOLD = 1
K_ATT = 0.06  # Attractive force constant
K_REP = 2.0  # Repulsive force constant


class PosePointRPY:
    def __init__(self, x, y, z, roll, pitch, yaw):
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.pos = PoseStamped()
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rollRad = math.radians(roll)
        pitchRad = math.radians(pitch)
        yawRad = math.radians(yaw)
        quaternion = transformations.quaternion_from_euler(rollRad, pitchRad, yawRad)
        self.pos.pose.orientation.x = quaternion[0]
        self.pos.pose.orientation.y = quaternion[1]
        self.pos.pose.orientation.z = quaternion[2]
        self.pos.pose.orientation.w = quaternion[3]
   
class Status(Enum):
    Takeoff = 0
    FlyMode = 1

class Avoidance:

    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        
        self.sub_cloud = rospy.Subscriber(
            '/iris/camera/depth/points', pc2.PointCloud2, self.cloud_callback, queue_size=1)

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)

        fixed_yaw = 180
        dist_factor = -10
        self.avoid_position = PosePointRPY(1, 0, FLIGHT_ALT,  0, 0, fixed_yaw)
        self.fixed_positions = [
            PosePointRPY(dist_factor*0, 0, FLIGHT_ALT, 0, 0, fixed_yaw),
            PosePointRPY(dist_factor*1, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*2, 0, FLIGHT_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*3, 0, FLIGHT_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*4, 0, FLIGHT_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*5, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*6, 0, FLIGHT_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*7, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*8, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*9, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*10, 0, FLIGHT_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*11, 0, FLIGHT_ALT, 0, 0, fixed_yaw),
            PosePointRPY(0, 180, FLIGHT_ALT, 0, 0, fixed_yaw)
        ]
        self.vel = Twist()
        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(20)
        self.last_req = rospy.Time.now()
        self.bridge = CvBridge()
        self.mean_mat = None
        self.drone_position = np.array([self.local_pose.pose.position.x,
                                        self.local_pose.pose.position.y,
                                        self.local_pose.pose.position.z])
        self.clusters = []
        self.last_published = rospy.Time.now()
        self.flight_status = Status.Takeoff
        self.yaw_angle = 180
        self.local_yaw = 1
        self.obstacles = np.array([])
        self.pre_transformed = np.array([])
        self.mean_time = 0.0
        self.mean_time_callback = 0.0
        self.count_time = 0
        self.db_count_time = 0

    def poseStamped_to_rpy(self, source: PoseStamped):
        rpy = transformations.euler_from_quaternion([source.pose.orientation.x, source.pose.orientation.y,
                    source.pose.orientation.z, source.pose.orientation.w])

        result = PosePointRPY(source.pose.position.x, source.pose.position.y, source.pose.position.z,
                    rpy[0], rpy[1], rpy[2])        

        return result
    
    def positionCallback(self, msg: PoseStamped):
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation = msg.pose.orientation
        rpy = transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,
                    msg.pose.orientation.z,msg.pose.orientation.w])
        self.local_yaw = rpy[2]
        self.drone_position = np.array([self.local_pose.pose.position.x,
                                        self.local_pose.pose.position.y,
                                        self.local_pose.pose.position.z])

    def state_callback(self, msg):
        self.current_state = msg
        
    def cloud_callback(self, cloud_msg):
        start_time = rospy.get_time()
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform("odom", "camera_link", rospy.Time(0), rospy.Duration(0, int( 4*10E8)))

        # Read points first
        points = list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
        points = points[::1000]
        points = [point for point in points if point[2] <= 30]
        # Convert the downsampled points back to a PointCloud2 message
        downsampled_cloud = pc2.create_cloud_xyz32(cloud_msg.header, points)

        # Transform the downsampled cloud
        transformed_cloud = do_transform_cloud(downsampled_cloud, transform)
        transformed_points = list(pc2.read_points(transformed_cloud, skip_nans=True, field_names=("x", "y", "z")))
        transformed_points = [point for point in transformed_points if point[2] >= 3]
        self.obstacles = np.array(transformed_points)
        self.publish_clusters(transformed_points)

        end_time = rospy.get_time()
        self.mean_time_callback += (end_time - start_time)
        self.count_time += 1

        if np.size(self.obstacles) > 0:
            self.clusters = self.group_points(self.obstacles)

    def group_points(self, cloud_points, eps=0.5, min_samples=20, max_distance=30):
        
        distances = np.linalg.norm(cloud_points - self.drone_position, axis=1)
        cloud_points = cloud_points[distances <= max_distance]
        self.publish_clusters(cloud_points)
        # Apply a pre-processing step to reduce the number of points
        # cloud_points = cloud_points[::15]  # Take every nth point


        start_time = rospy.get_time()
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(cloud_points)
        end_time = rospy.get_time()
        rospy.loginfo(f"DBSCAN fitting took {end_time - start_time} seconds")
        self.mean_time += (end_time - start_time)
        self.count_time += 1
        # Get the labels of the clusters
        labels = db.labels_

        # Convert labels to a list
        labels_list = labels.tolist()

        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels_list)) - (1 if -1 in labels_list else 0)

        # Create a list to hold the centroid and dispersion of each cluster
        clusters = [(np.mean(cloud_points[labels == i], axis=0)) for i in range(n_clusters)]
        rospy.loginfo(f"Number of Clusters: {len(clusters)}")
        if len(clusters) > 0:
            rospy.loginfo(f"\n Example {clusters[0]}")
            self.publish_clusters(clusters)
        return clusters

    def compute_repulsive_force(self):
        repulsive_force = np.zeros(3)

        if self.clusters:
            for obstacle, std_dev in self.clusters:
                # # Add the drone's position to the obstacle's position
                # obstacle += self.drone_position

                # Compute the distance between the drone and the obstacle
                distance = np.linalg.norm(self.drone_position - obstacle)

                # Compute the direction vector away from the obstacle
                direction_vector = (self.drone_position - obstacle) / distance

                # Compute repulsive force using the inverse square law, adjusted by the standard deviation
                repulsive_force += ((K_REP / distance**2) * (1 + std_dev)) * direction_vector

        return repulsive_force

    def compute_attractive_force(self):
        goal_pos = np.array([self.fixed_positions[-1].pos.pose.position.x,
                             self.fixed_positions[-1].pos.pose.position.y,
                             self.fixed_positions[-1].pos.pose.position.z])
        current_pos = np.array([self.local_pose.pose.position.x,
                                self.local_pose.pose.position.y,
                                self.local_pose.pose.position.z])

        attractive_force = K_ATT * (goal_pos - current_pos)
        return attractive_force

    def compute_height_force(self):
        force_factor = 1
        height_difference = FLIGHT_ALT - self.local_pose.pose.position.z
        z_force = height_difference * force_factor

        return z_force

    def potential_fields_avoidance(self):
        twist_msg = Twist()

        attractive_force = self.compute_attractive_force()
        repulsive_force = self.compute_repulsive_force()

        total_force = attractive_force + repulsive_force
        self.yaw_angle = math.atan2(attractive_force[1], attractive_force[0])

        yaw_error = self.yaw_angle - self.local_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        z_force = self.compute_height_force()

        twist_msg.linear.x = total_force[0]
        twist_msg.linear.y = total_force[1]
        twist_msg.linear.z = total_force[2] + z_force

        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = yaw_error * 0.3


        # rospy.loginfo(f"\n Atractive: \n x: {attractive_force[0]}, y: {attractive_force[1]}, z: {attractive_force[2]} \n Repulsive:\n x: {repulsive_force[0]}, y: {repulsive_force[1]}, z: {repulsive_force[2]} \n Value of Total Force: {total_force} \n")
        if rospy.Time.now() - self.last_published > rospy.Duration(0, 200000000):
            # self.pub_vel.publish(twist_msg)
            self.last_published = rospy.Time.now()


    def spin(self):
        self.potential_fields_avoidance()


    def publish_clusters(self, clusters):
        marker_array = MarkerArray()

        for i, centroid in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            scale_factor = 0.8

            marker.scale.x = scale_factor
            marker.scale.y = scale_factor
            marker.scale.z = scale_factor

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.id = i

            marker_array.markers.append(marker) # type: ignore

        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('avoidance_node') 
    avoider = Avoidance()
    rospy.loginfo("Marker view node initiated")

    while(not rospy.is_shutdown()):
        avoider.spin()
    if avoider.db_count_time > 0:
        avg_time = avoider.mean_time / avoider.db_count_time
        rospy.loginfo(f"Average time taken by DBSCAN: {avg_time} seconds")
    if avoider.count_time > 0:
        avg_time = avoider.mean_time_callback / avoider.count_time
        rospy.loginfo(f"Average time taken by Callback: {avg_time} seconds")
 