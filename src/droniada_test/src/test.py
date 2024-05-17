#!/usr/bin/env python3
from re import A
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import mavros_msgs.srv
from nav_msgs.msg import Odometry
import math
from tf import transformations
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

START_ALT = 10 # alt for drone flight
M_PI = 3.14159265359
TARGET_ANGL = 10 * (M_PI / 180.0)
POS_TRESHOLD =  0.0005

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


class Test:
    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.pub_pose = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.pub_vel = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        
        self.max_hor_vel = rospy.ServiceProxy(
            '/mavros/param/set', mavros_msgs.srv.ParamSet)

        self.global_pos = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.globalPositionCallback)

        self.pub_pose = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_callback)

        self.avoid_position = PosePointRPY(1, 0, START_ALT,  0, 0, M_PI)
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
        self.take_off_flag = False
        self.fixed_cords_list = []
        self.rate = rospy.Rate(20)
        self.state = 0
        self.cords = NavSatFix()



    def poseStamped_to_rpy(self, source: PoseStamped):
        rpy = transformations.euler_from_quaternion([source.pose.orientation.x, source.pose.orientation.y,
                    source.pose.orientation.z, source.pose.orientation.w])

        result = PosePointRPY(source.pose.position.x, source.pose.position.y, source.pose.position.z,
                    rpy[0], rpy[1], rpy[2])        

        return result
    

    def set_elements_positions(self):
        lat = self.cords.latitude
        lon = self.cords.longitude
        alt = self.cords.altitude  
        # Approximate conversion factors

        METERS_TO_LAT = 1 / 111111
        METERS_TO_LON = 1 / 111111
        header = rospy.Header()
        header.frame_id = "base_link"
        elements = [
            NavSatFix(header = header, latitude=lat, longitude=lon, altitude=alt + START_ALT),  # Starting point
            NavSatFix(header = header, latitude=lat + 3 * METERS_TO_LAT, longitude=lon + 3 * METERS_TO_LON, altitude=alt + START_ALT),  # "left_lower_ball"
            NavSatFix(header = header, latitude=lat + 7 * METERS_TO_LAT, longitude=lon + 3 * METERS_TO_LON, altitude=alt + START_ALT),  # "middle_lower_ball"
            NavSatFix(header = header, latitude=lat + 11 * METERS_TO_LAT, longitude=lon + 3 * METERS_TO_LON, altitude=alt + START_ALT),  # "right_lower_ball"
            NavSatFix(header = header, latitude=lat + 3 * METERS_TO_LAT, longitude=lon + 7 * METERS_TO_LON, altitude=alt + START_ALT),  # "left_central_ball"
            NavSatFix(header = header, latitude=lat + 7 * METERS_TO_LAT, longitude=lon + 7 * METERS_TO_LON, altitude=alt + START_ALT),  # "middle_central_ball"
            NavSatFix(header = header, latitude=lat + 11 * METERS_TO_LAT, longitude=lon + 7 * METERS_TO_LON, altitude=alt + START_ALT),  # "right_central_ball"
            NavSatFix(header = header, latitude=lat + 3 * METERS_TO_LAT, longitude=lon + 11 * METERS_TO_LON, altitude=alt + START_ALT),  # "left_upper_ball"
            NavSatFix(header = header, latitude=lat + 7 * METERS_TO_LAT, longitude=lon + 11 * METERS_TO_LON, altitude=alt + START_ALT),  # "middle_upper_ball"
            NavSatFix(header = header, latitude=lat + 11 * METERS_TO_LAT, longitude=lon + 11 * METERS_TO_LON, altitude=alt + START_ALT),  # "right_upper_ball"
            NavSatFix(header = header, latitude=lat + 7 * METERS_TO_LAT, longitude=lon + 27 * METERS_TO_LON, altitude=alt + START_ALT),  # "barrel"
        ]
        rospy.loginfo(f"{elements}")
        return elements
    
    def globalPositionCallback(self, msg: NavSatFix):
        self.cords = msg

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

    def update_state(self):
        if self.state < len(self.fixed_cords_list):
            self.state += 1
        else:
            rospy.signal_shutdown("Mission completed") 

    # def publish_pose(self, goal: PosePointRPY):
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     pose.pose.position.x = goal.pos.pose.position.x
    #     pose.pose.position.y = goal.pos.pose.position.y
    #     pose.pose.position.z = goal.pos.pose.position.z
    #     pose.pose.orientation.x = goal.pos.pose.orientation.x
    #     pose.pose.orientation.y = goal.pos.pose.orientation.y
    #     pose.pose.orientation.z = goal.pos.pose.orientation.z
    #     pose.pose.orientation.w = goal.pos.pose.orientation.w
    #     self.pub_pose.publish(pose)

    def publish_pose(self):
        goal = PositionTarget()
        goal.header = self.fixed_cords_list[self.state].header
        goal.pose.position.latitude = self.fixed_cords_list[self.state].latitude
        goal.pose.position.longitude = self.fixed_cords_list[self.state].longitude
        goal.pose.position.altitude = self.fixed_cords_list[self.state].altitude

        self.pub_pose.publish(goal)

    # def position_check(self):
    #     dx = self.cords.latitude - self.fixed_cords_list[self.state].latitude
    #     dy = self.cords.longitude - self.fixed_cords_list[self.state].longitude
    #     dz = self.cords.altitude - self.fixed_cords_list[self.state].altitude
    #     dist2 = dx**2 + dy**2 + dz**2
    #     return dist2 < POS_TRESHOLD**2
            # return False

        # curr_orient = self.local_pose.pose.orientation
        # curr_euler = transformations.euler_from_quaternion(
        #     [curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])
        # cr = abs(curr_euler[0] - math.radians(goal.roll))< TARGET_ANGL
        # cp = abs(curr_euler[1] - math.radians(goal.pitch)) < TARGET_ANGL
        # cy = abs(curr_euler[2] - math.radians(goal.yaw)) < TARGET_ANGL
        
        # in_range = cr and cp and cy

        # return in_range 

    def position_check(self):
        lat1, lon1 = math.radians(self.cords.latitude), math.radians(self.cords.longitude)
        lat2, lon2 = math.radians(self.fixed_cords_list[self.state].latitude), math.radians(self.fixed_cords_list[self.state].longitude)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        R = 6371.0

        distance = R * c


        dist3d = math.sqrt(distance**2 + (self.cords.altitude - self.fixed_cords_list[self.state].altitude)**2)

        pos_treshold_km = POS_TRESHOLD / 1000.0

        return dist3d < pos_treshold_km


    def spin(self):
        if self.position_check():
            rospy.loginfo(f"Desired Position [NUM: {self.state}] Reached ALT:{self.fixed_cords_list[self.state].altitude} LON:{self.fixed_cords_list[self.state].longitude} ALT:{self.fixed_cords_list[self.state].altitude}")
            self.update_state()
            self.publish_pose()
        else:
            self.publish_pose()
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('test_node') 
    tester = Test()
    rospy.loginfo("Test node initiated")

    while(not rospy.is_shutdown() and not tester.current_state.connected):
        tester.rate.sleep()
    tester.fixed_cords_list = tester.set_elements_positions()
    rospy.loginfo("Points updated")
    for i in range(10):   
        if(rospy.is_shutdown()):
            break
        tester.publish_pose()
        tester.rate.sleep()

    tester.offb_set_mode.custom_mode = 'OFFBOARD'
    tester.arm_cmd.value = True

    tester.last_req = rospy.Time.now()  
    rospy.loginfo("Preparing to takeoff")
    while(not rospy.is_shutdown()):
        tester.switch_to_offboard()
        tester.spin()
        if tester.state == 10:
            rospy.signal_shutdown("Mission completed") 

