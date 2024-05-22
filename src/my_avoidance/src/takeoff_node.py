#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf import transformations


START_ALT = 10 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 1 * (M_PI / 180.0)
POS_TRESHOLD = 0.3

class TakeOff:
    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_callback)

        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(20)
        self.current_state = State()
        self.offb_set_mode = SetModeRequest()
        self.arm_cmd = CommandBoolRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True
        self.last_req = rospy.Time.now()

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

    def publish_takeoff_point(self):
        start_pose =  PoseStamped()
        start_pose.pose.position.x = self.local_pose.pose.position.x
        start_pose.pose.position.y = self.local_pose.pose.position.x
        start_pose.pose.position.z = START_ALT
        start_pose.pose.orientation = self.local_pose.pose.orientation
        self.pub_pose.publish(start_pose)


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