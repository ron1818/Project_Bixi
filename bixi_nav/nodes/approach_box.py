#!/usr/bin/env python

""" reinaldomaslim
26-03-2017
"""

import rospy
import actionlib
import numpy as np
import math
import actionlib

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ApproachBox(object):
    x0, y0, yaw0= 0, 0, 0
    boxes=list()
    next_goal=Pose()

    def __init__(self, nodename):

        rospy.init_node(nodename, anonymous=False)

        #wait for odom
        self.odom_received = False
        rospy.wait_for_message("/odometry/filtered", Odometry)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=50)
        while not self.odom_received:
            rospy.sleep(1)
        print("odom received")

        rospy.Subscriber("/box", Pose, self.boxCallback, queue_size = 50)

        self.box_target_pub = rospy.Publisher('/next_box', Pose, queue_size=5)

        #Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # * Wait 60 seconds for the action server to become available
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        rate=rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.publishGoal()

            rate.sleep()

    def publishGoal(self):
        goal=self.findNextGoal()
        q_angle=quaternion_from_euler(0, 0, goal[2])
        q = Quaternion(*q_angle)
        self.next_goal=Pose(Point(goal[0], goal[1], 0), q)
        self.box_target_pub.pub(self.next_goal)
            

    def findNextGoal(self):
        distance=1000
        for i in range(len(self.boxes)):
            d=math.sqrt((self.x0-self.boxes[i][0])**2+(self.y0-self.boxes[i][1])**2)
            if d<distance:
                distance=d
                goal=self.boxes[i]

        return goal

    def boxCallback(self, msg):
        #every new box, check with stack if any available  
        xb, yb= msg.pose.position.x, msg.pose.position.y
        _, _, self.yawb = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

        for i in range(len(self.boxes)):
            if self.isWithin(xb, self.boxes[i][0]) and self.isWithin(yb, self.boxes[i][1]):
                #same box
                self.boxes[i]=[(xb+self.boxes[i][0])/2, (yb+self.boxes[i][1])/2, (yawb+self.boxes[i][2)/2] 
            else:
                #different box
                self.boxes.append([xb, yb, yawb])

    def isWithin(self, x1, x2):
        radius=0.2
        if abs(x1-x2)<radius:
            return True
        else:
            return False

    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
  

if __name__ == '__main__':
    try:
        ApproachBox(nodename="approach_box")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Boxes approaching finished.")
