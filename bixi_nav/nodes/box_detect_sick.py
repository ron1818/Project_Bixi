#!/usr/bin/env python

""" Reinaldo
    07-04-17
    calibrated for SICK lms100 laser 270 range upside down
"""
import rospy
import actionlib
import numpy as np
import math
import cv2

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker


class DetectSquare(object):
    x0, y0, yaw0= 0, 0, 0
    currentScan=LaserScan()
    box_length=0.2
    isUpsideDown=True

    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
    
        self.initMarker()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size = 50)

        rate=rospy.Rate(10)
    
        while not rospy.is_shutdown():


            rate.sleep()

    def scanCallback(self, msg):


        laserLabel=list()
        count=0
        members=0

        if self.isUpsideDown is True:
            for i in range(1, len(msg.ranges)):
                
                if abs(msg.ranges[i]-msg.ranges[i-1])>0.05:
                    
                    if members<5:
                        members=0
                        continue
                    members=0
                    laserLabel.append([count, i])
                    count=i
                members+=1

        self.boxes_position=self.detectBox(msg, laserLabel)
        self.printBox(self.boxes_position)



    def detectBox(self, msg, laserLabel):
        tolerance=0.05
        boxes=list()

        for x in laserLabel:
            #check their length 
            theta=abs((x[1]-x[0])*msg.angle_increment)
            l1=msg.ranges[x[0]]
            l2=msg.ranges[x[1]]

            length=math.sqrt(l1**2+l2**2-2*l1*l2*math.cos(theta))
            #print(length)
            if length>(self.box_length-tolerance) and length<(self.box_length+tolerance):
                alpha1=3*math.pi/4-x[0]*msg.angle_increment
                x1, y1=l1*math.cos(alpha1), l1*math.sin(alpha1)

                alpha2=3*math.pi/4-x[1]*msg.angle_increment
                x2, y2=l2*math.cos(alpha2), l2*math.sin(alpha2)

                x_mid, y_mid= self.x0+(x1+x2)/2, self.y0+(y1+y2)/2

                boxes.append([x_mid, y_mid])

        return boxes

    def printBox(self, box):
        self.boxes.points=list()
        if box is None:
            return

        for x in box:
            #markerList store points wrt 2D world coordinate
            
            p=Point()

            p.x=x[0]
            p.y=x[1]
            p.z=0

            self.boxes.points.append(p)
            self.boxes_pub.publish(self.boxes)



    def initMarker(self):
        # Set up our waypoint markers
        print("initializing markers")
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'boxes'
        marker_id = 0
        marker_color = {'r': 0.7, 'g': 0.5, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.boxes_pub = rospy.Publisher('boxes_marker', Marker, queue_size=5)

        # Initialize the marker points list.
        self.boxes = Marker()
        self.boxes.ns = marker_ns
        self.boxes.id = marker_id
        # self.markers.type = Marker.ARROW
        self.boxes.type = Marker.CUBE_LIST
        self.boxes.action = Marker.ADD
        self.boxes.lifetime = rospy.Duration(marker_lifetime)
        self.boxes.scale.x = marker_scale
        self.boxes.scale.y = marker_scale
        self.boxes.scale.z = marker_scale
        self.boxes.color.r = marker_color['r']
        self.boxes.color.g = marker_color['g']
        self.boxes.color.b = marker_color['b']
        self.boxes.color.a = marker_color['a']

        self.boxes.header.frame_id = 'odom'
        self.boxes.header.stamp = rospy.Time.now()
        self.boxes.points = list()

    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
  

if __name__ == '__main__':
    try:
        DetectSquare(nodename="detect_square")
    except rospy.ROSInterruptException:
        rospy.loginfo("Boxes detection finished.")
