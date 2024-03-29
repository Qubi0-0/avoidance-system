from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
# import imutils
# import math

HORIZONTAL_FOV = 1.02974
X_RES = 640
Y_RES = 480


class Tester:


    def __init__(self):
        
        self.last_depth = None
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)
        self.mean_mat = None
        self.img_spots = [[60,80], [60,240], [60,400], [60,560],
                          [180,80],[180,240],[180,400],[180,560],
                          [300,80],[300,240],[300,400],[300,560],
                          [420,80],[420,240],[420,400],[420,560]]
        self.sub_roi = rospy.Subscriber(
            '/ris/camera/depth/image_raw', Image, self.depth_callback, queue_size=1)

        self.sub_img = rospy.Subscriber(
            '/iris/camera/rgb/image_raw', Image, self.img_callback, queue_size=1)

    def split(self, array, nrows, ncols):
        """Split a matrix into sub-matrices."""
        r, h = array.shape
        return (array.reshape(h//nrows, nrows, -1, ncols)
                    .swapaxes(1, 2)
                    .reshape(-1, nrows, ncols))

    def mean_count(self, mat):
        dist_mean = [np.mean(sub_m) for sub_m in mat]
        return dist_mean

    def depth_callback(self, msg: Image):
        self.last_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1').copy()
        mat = self.split(self.last_depth,80,60)
        self.mean_mat = self.mean_count(mat) 
        self.img_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='8UC1').copy()

    def show_rgb_img(self):
        if self.mean_mat is not None:
            for i in range(0,16):
                self.img = cv.putText(self.img,str(self.mean_mat[i]),np.flip(self.img_spots[i]),cv.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0))
        cv.imshow("frame",self.img)
        if self.last_depth is not None:
            cv.imshow("depth",self.last_depth)

    def img_callback (self, msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgra8').copy()

    def show_img(self):
        self.show_rgb_img()
        cv.waitKey(1)

    def spin(self):
        self.show_img()
        self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('depth_node') 
    depth_client = Tester()
    rospy.loginfo("Tester node initiated")


    for i in range(10):   
        if(rospy.is_shutdown()):
            break
        depth_client.rate.sleep()
    
    while(not rospy.is_shutdown()):
        depth_client.spin()