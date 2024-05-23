#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import mavros_msgs.srv
from nav_msgs.msg import Odometry
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler


START_ALT = 8 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.3

class TakeOff:
    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        
        self.pub_sub = rospy.Subscriber("potential_twist", Twist, self.twist_callback)

        self.waypoint_sub = rospy.Subscriber(
            "drone_tracking/waypoint", PoseStamped, self.waypointCallback)

        self.max_hor_vel = rospy.ServiceProxy(
            '/mavros/param/set', mavros_msgs.srv.ParamSet)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_callback)

        self.vel = None
        self.odom = Odometry()
        self.local_pose = PoseStamped()
        self.waypoint = None
        self.rate = rospy.Rate(20)
        self.current_state = State()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True
        self.last_req = rospy.Time.now()
        self.take_off_flag = False

    def positionCallback(self, msg: PoseStamped):
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation = msg.pose.orientation
        rpy = transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,
                    msg.pose.orientation.z,msg.pose.orientation.w])
        self.local_yaw = rpy[2]

    def waypointCallback(self, msg: PoseStamped):
        self.waypoint = PoseStamped()
        self.waypoint.pose.position.x = msg.pose.position.x
        self.waypoint.pose.position.y = msg.pose.position.y
        self.waypoint.pose.position.z = msg.pose.position.z
        # Convert quaternion to Euler angles
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Subtract 90 degrees from yaw
        yaw += M_PI/2

        # Convert back to quaternion
        new_orientation_q = quaternion_from_euler(roll, pitch, yaw)

        # Assign the new orientation to the waypoint
        self.waypoint.pose.orientation.x = new_orientation_q[0]
        self.waypoint.pose.orientation.y = new_orientation_q[1]
        self.waypoint.pose.orientation.z = new_orientation_q[2]
        self.waypoint.pose.orientation.w = new_orientation_q[3]
        
    def twist_callback(self, msg: Twist):
        self.vel = Twist()
        self.vel = msg

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

    def spin_once(self):
        if self.vel is not None:
            self.pub_vel.publish(self.vel)
        elif self.waypoint is not None:
            self.pub_pose.publish(self.waypoint)
        else:
            rospy.loginfo("No control messages sent!")


if __name__ == '__main__':
    rospy.init_node('Mother_node') 
    avoider = TakeOff()
    rospy.loginfo("Control node initiated")

    while(not rospy.is_shutdown() and not avoider.current_state.connected):
        avoider.rate.sleep()

    for i in range(10):   
        if(rospy.is_shutdown()):
            break
        avoider.rate.sleep()

    avoider.offb_set_mode.custom_mode = 'OFFBOARD'
    avoider.arm_cmd.value = True

    avoider.last_req = rospy.Time.now()  
    while(not rospy.is_shutdown()):
        avoider.switch_to_offboard()
        avoider.spin_once()

            
