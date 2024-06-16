#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher('drone_path', Path, queue_size=10)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.path = Path()
        self.path.header.frame_id = "local_origin"
        # self.path.header.frame_id = "odom"

    def pose_callback(self, msg):
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(msg)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('path_publisher')
    pp = PathPublisher()
    rospy.spin()
