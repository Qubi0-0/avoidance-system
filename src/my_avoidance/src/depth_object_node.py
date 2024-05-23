from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import imutils
import math

HORIZONTAL_FOV = 1.02974
X_RES = 640
Y_RES = 480


class Tester:


    def __init__(self):
        
        self.last_depth = None
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)
        self.img_spots = [[60,80], [60,240], [60,400], [60,560],
                          [180,80],[180,240],[180,400],[180,560],
                          [300,80],[300,240],[300,400],[300,560],
                          [420,80],[420,240],[420,400],[420,560]]
        self.sub_roi = rospy.Subscriber(
            'camera/depth/image_raw', Image, self.depth_callback, queue_size=1)

        self.sub_img = rospy.Subscriber(
            'camera/rgb/image_raw', Image, self.img_callback, queue_size=1)

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
        for i in range(0,16):
            self.img = cv.putText(self.img,str(self.mean_mat[i]),np.flip(self.img_spots[i]),cv.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0))
        cv.imshow("frame",self.img)
        cv.imshow("depth",self.last_depth)



    def img_prepare(self):
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
                # set values as what you need in the situation
                cX, cY = 0, 0

            self.point.append([cX,cY])
            self.areas.append(area)

            cv.drawContours(self.img, [c], -1, (0, 255, 0), 2)
            cv.circle(self.img, (cX, cY), 7, (255, 255, 255), -1)
        if len(self.areas) != 0:
            idx = np.argmax(self.areas, axis=0)
            dist_x = self.calc_next_pose(idx)
        else:
            dist_x = 0.5

        if len(self.areas) != 0:         
            cv.drawContours(self.img, [cnts[idx]], -1, (0, 255, 0), 2)
            cv.circle(self.img, (self.point[idx][0], self.point[idx][1]), 7, (0, 0, 255), -1)
            cv.putText(self.img, f"Direction", (self.point[idx][0] + 20,
                                                                    self.point[idx][1] + 20),
            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),1) 

            cv.putText(self.img, f"Dist change: {dist_x}", (self.point[idx][0] - 20,
                                                                    self.point[idx][1] - 20),
            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),1) 
            cv.imshow("frame",self.img)

        if abs(dist_x) < 0.1:
            dist_x = 0
  
    def calc_next_pose(self,idx):
        # if self.point[idx][0] <= 480 or self.point[idx][1] < 480:
            # horizontal_length = 2 * self.img_depth[self.point[idx][0],
                                # self.point[idx][1]] * math.tan((HORIZONTAL_FOV /2))
        # else: 
        horizontal_length = 2 * 18 * math.tan((HORIZONTAL_FOV /2))
        scale = horizontal_length / X_RES
        dist_x = (self.point[idx][0] - X_RES//2) * scale
        return dist_x

    def img_callback (self, msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgra8').copy()

    def show_img(self):
        # self.show_rgb_img()
        self.img_prepare()
        # cv.imshow("means",self.prepared)
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