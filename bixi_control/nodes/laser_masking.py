#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo, LaserScan, CompressedImage


import cv2
from cv_bridge import CvBridge, CvBridgeError

class LaserMasking(object):


    def __init__(self):
        
        print("starting laser cam masking")
        rospy.init_node('laser_mask', anonymous=True)

        self.img_received=False 
        self.scan_received=False
        self.image_pub =rospy.Publisher("/lasermasked/image", Image, queue_size=10)

        rospy.wait_for_message("/usb_cam/image_raw", Image)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback, queue_size = 50)
        while not self.img_received:
            rospy.sleep(1)
        print("image received")


        rospy.wait_for_message("/scan", LaserScan)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 50)
        while not self.scan_received:
            rospy.sleep(1)
        print("laser scan received")

        

        while not rospy.is_shutdown():
            rospy.sleep(1)


    def img_callback(self, msg):


        #### direct conversion to CV2 ####
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.img_shape=image_np.shape
        self.img_received=True

        if not self.scan_received:
            return
        #print(image_np.shape)
        xsize, ysize=image_np.shape[0], image_np.shape[1]
        #region_select=np.copy(image_np)
        region_select=cv2.flip(image_np,  1)
        XX, YY = np.meshgrid(np.arange(0, xsize), np.arange(0, ysize))

        for m in self.result_list:

            region_select[:, m[0]:m[1], :]=[0, 0, 0]

        cv2.imshow("Image window", region_select)
        cv2.waitKey(3)
        #### Create CompressedIamge ####
        new_img = Image()
        new_img.header.stamp = rospy.Time.now()

        new_img.data =CvBridge().cv2_to_imgmsg(region_select, "bgr8")
        # Publish new image
        self.image_pub.publish(new_img)


    def scan_callback(self, msg):


        if not self.img_received:
            return

        camera_half_angle=math.pi/3 #60 degree
        N=camera_half_angle/msg.angle_increment
        threshold_distance=1.7
        #return [x1, x2] pair values for close obstacles
        start_points=list()
        end_points=list()
        self.result_list=list()

        #change togglePoint to dbclustering

        for i in range(int(N), int(2*N-1)):
            if self.togglePoint(msg.ranges[i], msg.ranges[i+1], threshold_distance)==1:
                x=int((i-N)*self.img_shape[1]/N)
                start_points.append(x)
            elif self.togglePoint(msg.ranges[i], msg.ranges[i+1], threshold_distance)==-1:
                x=int((i-N)*self.img_shape[1]/N)
                end_points.append(x)
        if len(start_points)==0 and len(end_points)==0:
            return


        k=0

        if start_points[0]>end_points[0]:
            self.result_list.append([0, end_points[0]])
            k+=1

        for i in range(k, self.min(len(start_points), len(end_points))-k):
            self.result_list.append([start_points[i], end_points[i]])
        
        if start_points[len(start_points)-1]>end_points[len(end_points)-1]:
            self.result_list.append([start_points[len(start_points)-1], self.img_shape[1]])

        self.scan_received=True

    def min(self, a, b):
        if a>b:
            return b
        else:
            return a

    def togglePoint(self, a, b, d):
        #return -1 for starting(false, true), 0 for none, 1 for ending(true, false), start and end of black censored 
        if a>d and b<d:
            return -1
        elif a<d and b>d:
            return 1
        else:
            return 0


if __name__ == '__main__':
    try:
        laserMask=LaserMasking()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished")
