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
from sklearn.cluster import MeanShift
from enum import Enum


START_ALT = 5 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.1
DEPTH_TRESHOLD = 5   # threshold for depth camera in meters
HEIGHT_TRESHOLD = 1
K_ATT = 0.01  # Attractive force constant
K_REP = 0.005  # Repulsive force constant

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
        
        fixed_yaw = 180
        dist_factor = -10
        self.avoid_position = PosePointRPY(1, 0, START_ALT,  0, 0, fixed_yaw)
        self.fixed_positions = [
            PosePointRPY(dist_factor*0, 0, START_ALT, 0, 0, fixed_yaw),
            PosePointRPY(dist_factor*1, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*2, 0, START_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*3, 0, START_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*4, 0, START_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*5, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*6, 0, START_ALT, 0, 0, fixed_yaw), 
            PosePointRPY(dist_factor*7, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*8, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*9, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*10, 0, START_ALT, 0, 0, fixed_yaw),  
            PosePointRPY(dist_factor*11, 0, START_ALT, 0, 0, fixed_yaw),
            PosePointRPY(dist_factor*12, 0, START_ALT, 0, 0, fixed_yaw)
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
        self.yaw_angle = 0.0

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
        self.clusters = self.group_points(self.obstacles)
    
    def group_points(self, cloud_points, eps=0.5, min_samples=30):
        # Apply a pre-processing step to reduce the number of points
        cloud_points = cloud_points[::10]  # Take every 10th point

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
        
        return clusters

    # def group_points(self, cloud_points, bandwidth=0.5):
    #     # Apply a pre-processing step to reduce the number of points
    #     # cloud_points = cloud_points[::10]  # Take every 10th point

    #     # Apply MeanShift to cloud_points
    #     ms = MeanShift(bandwidth=bandwidth).fit(cloud_points)

    #     # Get the labels of the clusters
    #     labels = ms.labels_

    #     # Convert labels to a list
    #     labels_list = labels.tolist()

    #     # Number of clusters in labels, ignoring noise if present.
    #     n_clusters = len(set(labels_list)) - (1 if -1 in labels_list else 0)

    #     # Create a list to hold the centroid and dispersion of each cluster
    #     clusters = [(np.mean(cloud_points[labels == i], axis=0), np.std(cloud_points[labels == i])) for i in range(n_clusters)]
    #     rospy.loginfo(f"Number of Clusters : {clusters.__len__()}")
        
    #     return clusters

    def switch_to_offboard(self):
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
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

        start_pose =  PoseStamped()
        start_pose.pose.position.x = self.local_pose.pose.position.x
        start_pose.pose.position.y = self.local_pose.pose.position.x
        start_pose.pose.position.z = START_ALT

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

    def compute_repulsive_force(self):
        repulsive_force = np.zeros(3)

        if self.clusters:
            for obstacle, std_dev in self.clusters:
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

    def potential_fields_avoidance(self):
        vector = Vector3Stamped()

        attractive_force = self.compute_attractive_force()
        repulsive_force = self.compute_repulsive_force()

        total_force = attractive_force + repulsive_force

        # Ensure the drone does not lower its height
        if total_force[2] < 0 and self.local_pose.pose.position.z < START_ALT:
            total_force[2] = 0

        # Update the velocity based on the computed force
        vector.vector.x = total_force[0]
        vector.vector.y = total_force[1]
        vector.vector.z = total_force[2]

        # Compute the yaw angle
        self.yaw_angle = math.atan2(total_force[1], total_force[0])

        rospy.loginfo(f"Atractive: {attractive_force} \n Repulsive: {repulsive_force}")
        if rospy.Time.now() - self.last_published > rospy.Duration(0, 100000000):
            self.pub_accel.publish(vector) # Publish vector
            self.last_published = rospy.Time.now()  # Update the time of the last publish
    def spin(self):
        self.potential_fields_avoidance()


if __name__ == '__main__':
    rospy.init_node('avoidance_node') 
    avoider = Avoidance()
    rospy.loginfo("Avoidance node initiated")
    current_status = Status.Takeoff

    
    while(not rospy.is_shutdown() and not avoider.current_state.connected):
        avoider.rate.sleep()

    for i in range(10):   
        if(rospy.is_shutdown()):
            break
        avoider.publish_takeoff_point()
        avoider.rate.sleep()

    avoider.offb_set_mode.custom_mode = 'OFFBOARD'
    avoider.arm_cmd.value = True

    avoider.last_req = rospy.Time.now()  
    rospy.loginfo("Preparing to takeoff")
    while(not rospy.is_shutdown()):
        avoider.switch_to_offboard()
    
        if current_status == Status.FlyMode:
            avoider.spin()
        elif (avoider.local_pose.pose.position.z > START_ALT - HEIGHT_TRESHOLD) and current_status == Status.Takeoff:
            current_status = Status.FlyMode          
            rospy.loginfo("STATUS Change to FlyMode")
            
        else:
            avoider.publish_takeoff_point()
