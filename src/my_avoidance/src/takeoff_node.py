#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import mavros_msgs.srv
from nav_msgs.msg import Odometry
import math
from tf import transformations


START_ALT = 5 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.3


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


class TakeOff:
    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        
        self.max_hor_vel = rospy.ServiceProxy(
            '/mavros/param/set', mavros_msgs.srv.ParamSet)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_callback)

        fixed_yaw = 180
        self.avoid_position = PosePointRPY(1, 0, START_ALT,  0, 0, fixed_yaw)
        self.vel = Twist()
        self.odom = Odometry()
        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(20)
        self.current_state = State()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True
        self.last_req = rospy.Time.now()

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
        except rospy.ServiceException as e:
            print("Service max_horizontal_velocity (MPC_XY_VEL_MAX) call failed: %s" % e)

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
        self.set_horizontal_velocity(2)

        start_pose =  PoseStamped()
        start_pose.pose.position.x = self.local_pose.pose.position.x
        start_pose.pose.position.y = self.local_pose.pose.position.x
        start_pose.pose.position.z = START_ALT
        rollRad = 0 
        pitchRad = 0 
        yawRad = math.radians(240)
        quaternion = transformations.quaternion_from_euler(rollRad, pitchRad, yawRad)
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
        # if in_range:
            # rospy.loginfo(f"Desired Position Reached X:{self.fixed_positions[self.state].pos.pose.position.x} Y:{self.fixed_positions[self.state].pos.pose.position.y}")
        return in_range 


if __name__ == '__main__':
    rospy.init_node('takeoff_node') 
    avoider = TakeOff()
    rospy.loginfo("Takeoff node initiated")

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
        # Check if the drone has reached the desired altitude
        avoider.publish_takeoff_point()
        if avoider.local_pose.pose.position.z >= START_ALT - POS_TRESHOLD:
            rospy.loginfo("Takeoff complete, stopping node")
            break