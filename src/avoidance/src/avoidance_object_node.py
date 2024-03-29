#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
import mavros_msgs.srv
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2 as cv
import math
import imutils
import tf 
 


START_ALT = 1.5 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.1
DEPTH_TRESHOLD = 5   # threshold for depth camera in meters
HEIGHT_TRESHOLD = 0.2
HORIZONTAL_FOV = 1.02974
X_RES = 640
Y_RES = 480
DEPTH_DIST = 18
DIST_FACTOR = -5 #-5 -10
FIXED_YAW = -170

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
        quaternion = tf.transformations.quaternion_from_euler(rollRad, pitchRad, yawRad)
        self.pos.pose.orientation.x = quaternion[0]
        self.pos.pose.orientation.y = quaternion[1]
        self.pos.pose.orientation.z = quaternion[2]
        self.pos.pose.orientation.w = quaternion[3]

        

# Start coordinates:

    # x: -152.23712158203125
    # y: -37.693355560302734
    # z: 4.133914947509766


class Avoidance:

    """
    By computing data from depth_cameta tries to avoid collision with any found obstacle
    """

    def __init__(self):
        
        self.sub_pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.position_callback)

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        self.sub_roi = rospy.Subscriber(
            'camera/depth/image_raw', Image, self.depth_callback, queue_size=1)

        self.sub_img = rospy.Subscriber(
            'camera/rgb/image_raw', Image, self.img_callback, queue_size=1)

        self.max_hor_vel = rospy.ServiceProxy(
            '/mavros/param/set', mavros_msgs.srv.ParamSet)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)

        # for showing the values on RGB frame
        self.img_spots = [[60,80], [60,240], [60,400], [60,560],
                          [180,80],[180,240],[180,400],[180,560],
                          [300,80],[300,240],[300,400],[300,560],
                          [420,80],[420,240],[420,400],[420,560]]
        self.avoid_position = PosePointRPY(1, 0, START_ALT,  0, 0, FIXED_YAW)
        self.fixed_y_val = 0
        self.fixed_positions = [
          # PosePointRPY(x,y,z,roll,pitch,yaw) 
            PosePointRPY( DIST_FACTOR*0 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),
            PosePointRPY( DIST_FACTOR*1 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            PosePointRPY( DIST_FACTOR*2 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW), 
            PosePointRPY( DIST_FACTOR*3 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW), 
            PosePointRPY( DIST_FACTOR*4 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW), 
            PosePointRPY( DIST_FACTOR*5 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            PosePointRPY( DIST_FACTOR*6 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW), 
            PosePointRPY( DIST_FACTOR*7 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW)  
            # PosePointRPY( DIST_FACTOR*8 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*9 , self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*10, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*11, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),
            # PosePointRPY( DIST_FACTOR*12, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),
            # PosePointRPY( DIST_FACTOR*13, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*14, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW), 
            # PosePointRPY( DIST_FACTOR*15, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*16, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*17, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*18, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),  
            # PosePointRPY( DIST_FACTOR*19, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW),
            # PosePointRPY( DIST_FACTOR*20, self.fixed_y_val ,START_ALT, 0, 0, FIXED_YAW)
        ]
        self.vel = Twist()
        self.odom = Odometry()
        self.state = 0
        self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(20)
        self.current_state = State()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True
        self.last_req = rospy.Time.now()
        self.bridge = CvBridge()
        self.avoid_check = False
        self.mean_mat = None

    def poseStamped_to_rpy(self, source: PoseStamped):

        rpy = tf.transformations.euler_from_quaternion([source.pose.orientation.x,source.pose.orientation.y,
                    source.pose.orientation.z,source.pose.orientation.w])

        result = PosePointRPY(source.pose.position.x,source.pose.position.y,source.pose.position.z,
                    rpy[0],rpy[1],rpy[2])        

        return result

    def set_horizontal_velocity(self,max_velocity):
        rospy.wait_for_service('/mavros/param/set')
        try:
            self.max_hor_vel(param_id="MPC_XY_VEL_ALL", value=mavros_msgs.msg.ParamValue(real=max_velocity))
            # rospy.loginfo(f"horizontal velocity set to: {max_velocity}")
        except rospy.ServiceException as e:
            print("Service max_horizontal_velocity (MPC_XY_VEL_MAX) call failed: %s" % e)

    def position_callback (self, msg: PoseStamped):
        "Gets the position of a drone and safes it to variable"
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation = msg.pose.orientation
        rpy = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,
                    msg.pose.orientation.z,msg.pose.orientation.w])
        self.local_yaw = rpy[2]

    def depth_callback(self, msg: Image):
        self.last_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1').copy()
        mat = self.split(self.last_depth,80,60)
        self.mean_mat = self.mean_count(mat) 

        self.img_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='8UC1').copy()

    def state_callback(self,msg):
        self.current_state = msg

    def img_callback (self, msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgra8').copy()

    def split(self, array, nrows, ncols):
        """Split a matrix into sub-matrices."""
        r, h = array.shape
        return (array.reshape(h//nrows, nrows, -1, ncols)
                    .swapaxes(1, 2)
                    .reshape(-1, nrows, ncols))

    def mean_count(self, mat):
        dist_mean = [np.mean(sub_m) for sub_m in mat]
        return dist_mean

    def show_rgb_img(self):
        for i in range(0,16):
            self.img = cv.putText(self.img,str(self.mean_mat[i]),np.flip(self.img_spots[i]),cv.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255))
        cv.imshow("frame",self.img)
        # cv.imshow("depth",self.last_depth)
        cv.waitKey(1)

    def switch_to_offboard(self):
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            self.last_req = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                self.last_req = rospy.Time.now()

    def publish_pose(self, goal: PosePointRPY):
        "Publishes the goal point for drone to move in"
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

        # Publish Velocity setpoint
        self.set_horizontal_velocity(2)

        start_pose =  PoseStamped()
        start_pose.pose.position.x = self.local_pose.pose.position.x
        start_pose.pose.position.y = self.local_pose.pose.position.x
        start_pose.pose.position.z = START_ALT
        rollRad = 0 
        pitchRad = 0 
        yawRad = math.radians(240)
        quaternion = tf.transformations.quaternion_from_euler(rollRad, pitchRad, yawRad)
        start_pose.pose.orientation.x = quaternion[0]
        start_pose.pose.orientation.y = quaternion[1]
        start_pose.pose.orientation.z = quaternion[2]
        start_pose.pose.orientation.w = quaternion[3]
        self.pub_pose.publish(start_pose)

    def position_check(self,goal: PosePointRPY):
        "Checks if drone already stopped at certain point"
        dx = self.local_pose.pose.position.x - goal.pos.pose.position.x
        dy = self.local_pose.pose.position.y - goal.pos.pose.position.y
        dz = self.local_pose.pose.position.z - goal.pos.pose.position.z
        dist2 = dx**2 + dy**2 + dz**2
        if dist2 > POS_TRESHOLD**2:
            return False

        curr_orient = self.local_pose.pose.orientation
        curr_euler = tf.transformations.euler_from_quaternion(
            [curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])
        cr = abs(curr_euler[0] - math.radians(goal.roll))< TARGET_ANGL
        cp = abs(curr_euler[1] - math.radians(goal.pitch)) < TARGET_ANGL
        cy = abs(curr_euler[2] - math.radians(goal.yaw)) < TARGET_ANGL
        
        in_range = cr and cp and cy
        if in_range:
            self.avoid_check = False
            rospy.loginfo(f"Desired Position Reached X:{self.fixed_positions[self.state].pos.pose.position.x} Y:{self.fixed_positions[self.state].pos.pose.position.y}")
        return in_range 

    def update_state(self):
        if self.state < len(self.fixed_positions):
            self.state += 1
            self.fixed_positions[self.state].pos.pose.position.y = self.local_pose.pose.position.y
            if self.state >= len(self.fixed_positions):
                self.state = len(self.fixed_positions) -1
        else:
            self.state = 1

    def calc_next_pose(self,idx):
        horizontal_length = 2 * DEPTH_DIST * math.tan((HORIZONTAL_FOV /2))
        scale = horizontal_length / X_RES
        dist_x = (self.point[idx][0] - X_RES//2) * scale
        return dist_x

    def pose_change_calc(self):
        # imgray = cv.cvtColor(self.img_depth, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(self.img_depth, 5, 20, cv.THRESH_BINARY)
        contours = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(contours)
        self.point = []
        self.areas = []
        for c in cnts:
            area = cv.contourArea(c)
                # compute the center of the contour
            M = cv.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                # set values as 0 in case of dividing by it
                cX, cY = 0, 0

            self.point.append([cX,cY])
            self.areas.append(area)
        if len(self.areas) != 0:
            idx = np.argmax(self.areas, axis=0)
            dist_x = self.calc_next_pose(idx)
        else:
            dist_x = -2
        if abs(dist_x) < 0.1:
            dist_x = 3

        self.avoid_dist = dist_x


    def avoid(self):
        rospy.loginfo("Possible collision detected!")
        middle_mean = self.mean_mat[18:21] + self.mean_mat[28:29] + self.mean_mat[34:37] + self.mean_mat[42:45]
        # if any(mean_win <= 5 for mean_win in middle_mean) and abs(self.avoid_dist) < 1:
            # self.fixed_y_val = self.local_pose.pose.position.y + 4
            # rospy.loginfo("fixed rout needed")
        # else:
            # self.fixed_y_val = self.local_pose.pose.position.y + self.avoid_dist
        changed_y_val = self.local_pose.pose.position.y + 0.3* self.avoid_dist

        self.avoid_position.pos.pose.position.x = self.fixed_positions[self.state].pos.pose.position.x
        self.avoid_position.pos.pose.position.y = changed_y_val
        self.avoid_position.pos.pose.position.z = START_ALT
        # self.fixed_positions.insert(self.state,self.avoid_position)
        self.fixed_positions[self.state] = self.avoid_position
        self.avoid_check = True
        self.fixed_y_val = changed_y_val
        self.rate.sleep()

    def spin(self):
        # self.show_rgb_img()
        mean_mat_up = self.mean_mat[0:49] # we don`t look on the bottom values of the camera
        self.pose_change_calc()
        if not any(sub_mat < DEPTH_TRESHOLD for sub_mat in mean_mat_up): # While depth camera doesn`t see close objects
            if self.position_check(self.fixed_positions[self.state]):
                self.update_state()
            self.publish_pose(self.fixed_positions[self.state])
        else:
            if self.position_check(self.fixed_positions[self.state]) or not self.avoid_check: # Checks if avoid position has been reached or new avoid position needed
                self.avoid()
            self.publish_pose(self.fixed_positions[self.state])

        self.rate.sleep()


if __name__ == '__main__':

    
    rospy.init_node('avoidance_node') 
    avoider = Avoidance()
    rospy.loginfo("Avoidance node initiated")


    while(not rospy.is_shutdown() and not avoider.current_state.connected):
        avoider.rate.sleep()

    # Send a few setpoints before starting
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

        if avoider.local_pose.pose.position.z > START_ALT - HEIGHT_TRESHOLD and avoider.local_pose.pose.position.z < START_ALT + HEIGHT_TRESHOLD:
            avoider.spin()
        else:
            avoider.publish_takeoff_point()


    