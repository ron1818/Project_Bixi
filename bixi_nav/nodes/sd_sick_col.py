#!/usr/bin/env python

""" Reinaldo
    3-11-16
    calibrated for SICK lms laser 270 range
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
        window_length=4

        resolution=0.01
        res=resolution*4
        size=window_length/resolution
        self.mid_point=int(size/2)
        #print(self.mid_point)

        laserGrid=np.zeros((size, size), dtype=np.uint8)

        if self.isUpsideDown is True:
            for i in range(len(msg.ranges)):
                
                if msg.ranges[i]<window_length/2:
                    theta=3*math.pi/4-i*msg.angle_increment
                    d=msg.ranges[i]
                    
                    #laser coordinate in grid units
                    x=int(d*math.cos(theta)/res)
                    y=int(d*math.sin(theta)/res)
                    

                    #pixel coordinate
                    px=self.mid_point-y
                    py=self.mid_point-x

                    laserGrid[px][py]=1
        else:
            for i in range(len(msg.ranges)-1, 0):
                
                if msg.ranges[i]<window_length/2:
                    theta=i*msg.angle_increment-3*math.pi/4
                    d=msg.ranges[i]
                    px=int(d*math.cos(theta)/resolution)+mid_point
                    py=int(d*math.sin(theta)/resolution)+mid_point

                    laserGrid[px][py]=1   
        higher_reso = cv2.pyrUp(cv2.pyrUp(laserGrid))
        self.boxes_position=self.detectBox(higher_reso, resolution)
        self.printBox(self.boxes_position)

    def detectBox(self, grid, resolution):
        origin=int(grid.shape[0]/2)
        #extract lines in rolling window
        rho = 1 # distance resolution in pixels of the Hough grid
        theta = np.pi/180 # angular resolution in radians of the Hough grid
        threshold = 8 # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 5 # minimum number of pixels making up a line
        max_line_gap = 20/resolution  # maximum gap in pixels between connectable line segments
        tolerance=0.01
        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(grid, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
        boxes=list()

        if lines is None:
            #print("none")
            return None

        for line in lines:
            for x1,y1,x2,y2 in line:
                
                length=(math.sqrt((x2-x1)**2+(y2-y1)**2))*resolution
                print(length)


                if length>(self.box_length-tolerance)and length<(self.box_length+tolerance):
                    #this is box's edge
                    
                    d=self.box_length/(2*resolution)
                    theta=math.atan2(y2-y1, x2-x1)#+math.pi/2 #angle of mid-center
                    #print(x1, y1)
                    #print(x2, y2)
                    y_mid=(y2+y1)/2
                    x_mid=(x2+x1)/2
                    #extract center
                    print(x_mid*resolution, y_mid*resolution, theta*180/math.pi)

                    del_y=d*math.sin(theta)
                    del_x=d*math.cos(theta)

                    if math.sqrt((x_mid+del_x-origin)**2+(y_mid+del_y-origin)**2)>math.sqrt((x_mid-origin)**2+(y_mid-origin)**2):
                        x_c=x_mid+del_x
                        y_c=y_mid+del_y
                    else:
                        x_c=x_mid-del_x
                        y_c=y_mid-del_y

                    #print(x_c, y_c)
                    #print(x_mid, y_mid)
                    #print((x_mid-origin)*resolution, (y_mid-origin)*resolution)
                    a=(x_c-origin)*resolution
                    b=(y_c-origin)*resolution

                    edge_x=self.x0+(origin-x_mid)*resolution
                    edge_y=self.y0+(origin-y_mid)*resolution
                    
                    #boxes.append([self.x0+a*math.sin(self.yaw0)+b*math.cos(self.yaw0), self.y0-a*math.cos(self.yaw0)+b*math.sin(self.yaw0)])
                    boxes.append([edge_x, edge_y])
        #y-axis
        #boxes.append([self.x0, self.y0+1])

        #print("boxes position:", boxes)
        #return global positions of boxes
        return boxes

    def pixToLaser(self, px, py):
        return self.mid_point+py, self.mid_point-px




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
