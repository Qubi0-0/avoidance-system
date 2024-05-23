#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
import mavros_msgs.srv
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import math
from tf import transformations
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray


FLIGHT_ALT = 10 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.1
DEPTH_TRESHOLD = 5   # threshold for depth camera in meters
HEIGHT_TRESHOLD = 1
K_ATT = 0.01  # Attractive force constant
K_REP = 1.0  # Repulsive force constant

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

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        self.max_hor_vel = rospy.ServiceProxy(
            '/mavros/param/set', mavros_msgs.srv.ParamSet)
        
        self.pub_accel = rospy.Publisher(
            "/mavros/setpoint_accel/accel", Vector3Stamped, queue_size=1)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        self.sub_cloud = rospy.Subscriber(
            '/iris/camera/depth/points', pc2.PointCloud2, self.cloud_callback, queue_size=1)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_callback)
        
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

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
        self.odom = Odometry()
        self.state = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(20)
        self.current_state = State()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True
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

    def poseStamped_to_rpy(self, source: PoseStamped):
        rpy = transformations.euler_from_quaternion([source.pose.orientation.x, source.pose.orientation.y,
                    source.pose.orientation.z, source.pose.orientation.w])

        result = PosePointRPY(source.pose.position.x, source.pose.position.y, source.pose.position.z,
                    rpy[0], rpy[1], rpy[2])        

        return result

    def set_horizontal_velocity(self, max_velocity):
        rospy.wait_for_service('/mavros/param/set')
        try:
            self.max_hor_vel(param_id="MPC_XY_VEL_ALL", value=mavros_msgs.msg.ParamValue(real=max_velocity)) # type: ignore
        except rospy.ServiceException as exception:
            print(f"Service max_horizontal_velocity (MPC_XY_VEL_MAX) call failed: {exception}")

    def positionCallback(self, msg: PoseStamped):
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation = msg.pose.orientation
        rpy = transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,
                    msg.pose.orientation.z,msg.pose.orientation.w])
        self.local_yaw = rpy[2]

    def state_callback(self, msg):
        self.current_state = msg
        
    def cloud_callback(self, cloud_msg):
        cloud_points = list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
        self.obstacles = np.array(cloud_points)

        # Group the points
        if cloud_points:
            self.clusters = self.group_points(self.obstacles)

    def switch_to_offboard(self, type):
        if(self.current_state.mode != type and (rospy.Time.now() - self.last_req) > rospy.Duration(5)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo(f"{type} enabled")
            
            self.last_req = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                self.last_req = rospy.Time.now()

    def publish_pose(self, goal: PosePointRPY):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = goal.pos.pose.position.x
        pose.pose.position.y = goal.pos.pose.position.y
        pose.pose.position.z = goal.pos.pose.position.z
        pose.pose.orientation.x = goal.pos.pose.orientation.x
        pose.pose.orientation.y = goal.pos.pose.orientation.y
        pose.pose.orientation.z = goal.pos.pose.orientation.z
        pose.pose.orientation.w = goal.pos.pose.orientation.w
        self.pub_pose.publish(pose)

    def publish_takeoff_point(self):
        # self.set_horizontal_velocity(2)
        self.flight_status = Status.Takeoff
        start_pose =  PoseStamped()
        start_pose.pose.position.x = self.local_pose.pose.position.x
        start_pose.pose.position.y = self.local_pose.pose.position.x
        start_pose.pose.position.z = FLIGHT_ALT
        
        quaternion = transformations.quaternion_from_euler(0, 0, self.yaw_angle)
        start_pose.pose.orientation.x = quaternion[0]
        start_pose.pose.orientation.y = quaternion[1]
        start_pose.pose.orientation.z = quaternion[2]
        start_pose.pose.orientation.w = quaternion[3]
        self.pub_pose.publish(start_pose)

    def position_check(self, goal: PosePointRPY):
        dx = self.local_pose.pose.position.x - goal.pos.pose.position.x
        dy = self.local_pose.pose.position.y - goal.pos.pose.position.y
        dz = self.local_pose.pose.position.z - goal.pos.pose.position.z
        dist2 = dx**2 + dy**2 + dz**2
        if dist2 > POS_TRESHOLD**2:
            return False

        curr_orient = self.local_pose.pose.orientation
        curr_euler = transformations.euler_from_quaternion(
            [curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])
        cr = abs(curr_euler[0] - math.radians(goal.roll)) < TARGET_ANGL
        cp = abs(curr_euler[1] - math.radians(goal.pitch)) < TARGET_ANGL
        cy = abs(curr_euler[2] - math.radians(goal.yaw)) < TARGET_ANGL
        
        in_range = cr and cp and cy
        if in_range:
            rospy.loginfo(f"Desired Position Reached X:{self.fixed_positions[self.state].pos.pose.position.x} Y:{self.fixed_positions[self.state].pos.pose.position.y}")
        return in_range 

    def updateState(self):
        if self.state < len(self.fixed_positions):
            self.state += 1
            self.fixed_positions[self.state].pos.pose.position.y = self.local_pose.pose.position.y
            if self.state >= len(self.fixed_positions):
                self.state = len(self.fixed_positions) -1
        else:
            self.state = 1

    def group_points(self, cloud_points, eps=0.5, min_samples=10, max_distance=30):
        # distances = np.linalg.norm(cloud_points, axis=1)

        # cloud_points = cloud_points[distances <= max_distance]

        # Apply a pre-processing step to reduce the number of points
        cloud_points = cloud_points[::10]  # Take every 5th point

        # Apply DBSCAN to cloud_points
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(cloud_points)

        # Get the labels of the clusters
        labels = db.labels_

        # Convert labels to a list
        labels_list = labels.tolist()

        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels_list)) - (1 if -1 in labels_list else 0)

        # Create a list to hold the centroid and dispersion of each cluster
        clusters = [(np.mean(cloud_points[labels == i], axis=0), np.std(cloud_points[labels == i])) for i in range(n_clusters)]
        rospy.loginfo(f"Number of Clusters: {len(clusters)} \n Example {clusters[0]}")
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
            self.pub_vel.publish(twist_msg)
            self.last_published = rospy.Time.now()


    def spin(self):
        self.potential_fields_avoidance()


    def publish_clusters(self, clusters):
        marker_array = MarkerArray()

        for i, (centroid, std_dev) in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
        
            # Add the drone's position to the centroid's position
            centroid += self.drone_position

            # Set the marker's position
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            factor = 0.1
            # Set the marker's size proportional to the standard deviation
            marker.scale.x = std_dev * factor
            marker.scale.y = std_dev * factor
            marker.scale.z = std_dev * factor

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
    rospy.loginfo("Avoidance node initiated")

    
    while(not rospy.is_shutdown() and not avoider.current_state.connected):
        avoider.rate.sleep()

    for i in range(10):   
        if(rospy.is_shutdown()):
            break
        avoider.publish_takeoff_point()
        avoider.rate.sleep()

    avoider.offb_set_mode.custom_mode = 'OFFBOARD'
    # avoider.offb_set_mode.custom_mode = 'POSCTL'
    avoider.arm_cmd.value = True

    avoider.last_req = rospy.Time.now()  
    rospy.loginfo("Preparing to takeoff")
    while(not rospy.is_shutdown()):
        avoider.switch_to_offboard(avoider.offb_set_mode.custom_mode)


        if avoider.flight_status == Status.FlyMode:
            avoider.spin()
        elif (avoider.local_pose.pose.position.z > FLIGHT_ALT - HEIGHT_TRESHOLD) and avoider.flight_status == Status.Takeoff:
            avoider.flight_status = Status.FlyMode          
            rospy.loginfo("STATUS Change to FlyMode")
            
        else:
            avoider.publish_takeoff_point()