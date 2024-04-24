#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from tf import transformations
import numpy as np
import math
from tf.transformations import quaternion_from_euler, quaternion_multiply

RATE = 10.0

class Transformer:

    def __init__(self):
        self.pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.positionCallback)

        self.local_pose = PoseStamped()
        self.rate = rospy.Rate(RATE)

        
    def publish_transform_camera(self):

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "drone_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        r = 90  * math.pi / 180
        p = 180 * math.pi / 180
        y = 90  * math.pi / 180

        q = quaternion_from_euler(r,p,y)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    def publish_transform_points(self):

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "point_link"
        r = 0  * math.pi / 180
        p = 180 * math.pi / 180
        y = 0  * math.pi / 180
        current_orientation = [self.local_pose.pose.orientation.x, 
                               self.local_pose.pose.orientation.y, 
                               self.local_pose.pose.orientation.z, 
                               self.local_pose.pose.orientation.w]
        q = quaternion_from_euler(r,p,y)
        new_orientation = quaternion_multiply(current_orientation, q)

        t.transform.translation.x = self.local_pose.pose.position.x
        t.transform.translation.y = self.local_pose.pose.position.y
        t.transform.translation.z = self.local_pose.pose.position.z
        t.transform.rotation.x = new_orientation[0]
        t.transform.rotation.y = new_orientation[1]
        t.transform.rotation.z = new_orientation[2]
        t.transform.rotation.w = new_orientation[3]

        br.sendTransform(t)

    def publish_transform_drone(self):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()


        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "drone_link"
        t.transform.translation.x = self.local_pose.pose.position.x
        t.transform.translation.y = self.local_pose.pose.position.y
        t.transform.translation.z = self.local_pose.pose.position.z
        t.transform.rotation.x = self.local_pose.pose.orientation.x
        t.transform.rotation.y = self.local_pose.pose.orientation.y
        t.transform.rotation.z = self.local_pose.pose.orientation.z
        t.transform.rotation.w = self.local_pose.pose.orientation.w

        br.sendTransform(t)


    def positionCallback(self, msg: PoseStamped):
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation = msg.pose.orientation
        


if __name__ == '__main__':
    rospy.init_node('transform_publisher_node')
    rospy.loginfo("Node has been initiated")
    transformer = Transformer()
    while not rospy.is_shutdown():
        transformer.publish_transform_camera()
        transformer.publish_transform_drone()
        transformer.publish_transform_points()
        transformer.rate.sleep()