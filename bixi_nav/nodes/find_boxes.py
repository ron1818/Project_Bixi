#!/usr/bin/env python

""" Reinaldo
    5-5-17
"""
import rospy
import actionlib
import numpy as np
import math
import cv2

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from sklearn.cluster import MeanShift, estimate_bandwidth

class IdentifyBox(object):
    x0, y0, yaw0= 0, 0, 0
    box_length=0.2
    edges=[]
    clustered_edges=[]
    box=[]
    n_edge=30
    cam_angle=math.pi/2

    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
    
        self.initMarker()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/edge", PoseStamped, self.edgeCallback, queue_size = 50)
        self.box_pose_pub=rospy.Publisher("/box", PoseStamped, queue_size=10)

        rate=rospy.Rate(10)
        msg=PoseStamped()
        msg.header.frame_id="odom"    

        while not rospy.is_shutdown():

            if self.box not None:
                for x in self.box:

                    msg.pose.position.x = x[0]
                    msg.pose.position.y = x[1]
                    q_angle = quaternion_from_euler(0, 0, x[2])
                    msg.pose.orientation = Quaternion(*q_angle)
                    self.box_pose_pub.publish(msg)

            rate.sleep()

    def edgeCallback(self, msg):
        #for a detected edge, add it into the list. If list is full, replace the first element.
        if len(self.edges)==n_edge:
            #remove the first element
            del self.edges[0]

        _, _, yaw_angle = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.edges.append([msg.pose.x, msg.pose.y, yaw_angle])


        #perform clustering to edges
        X=np.asarray(self.edges)
        bandwidth = estimate_bandwidth(X, quantile=0.4)
        ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        ms.fit(X)

        self.clustered_edges = ms.cluster_centers_

    def symbolCallback(self, msg):
        #msg.x is type 0-> left, 1->right, 2->hole
        #msg.z is the symbol's center angle wrt camera's field of view. 
        #heading in rad, cameras field of view=78 deg, camera's center heading as cam_angle

        sym_heading=self.yaw0+cam_angle-msg.z

        #match symbol with existing edges

        for edge in self.clustered_edges:
            theta=math.atan2(edge[0]-self.x0, edge[1]-self.y0)
            #if difference between theta and symbol heading within 5 degrees
            if abs(sym_heading-theta)<5*math.pi/180:
                #found the right edge, compute center of box
                center_x=edge[0]-self.box_length*math.cos(edge[2])/2
                center_y=edge[1]-self.box_length*math.sin(edge[2])/2

                if msg.x==0:
                    #left
                    direction=edge[2]-math.pi/2
                elif msg.x==1:
                    #right
                    direction=edge[2]+math.pi/2
                elif msg.x==2:
                    #center
                    direction=edge[2]

                self.box.append([center_x, center_y, direction])
                break

    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
  

if __name__ == '__main__':
    try:
        IdentifyBox(nodename="identify_box")
    except rospy.ROSInterruptException:
        rospy.loginfo("Boxes detection finished.")
