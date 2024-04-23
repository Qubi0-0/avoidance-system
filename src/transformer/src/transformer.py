#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_transform():

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 1.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        br.sendTransform(t)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('transform_publisher_node')
    publish_transform()