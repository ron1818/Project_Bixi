#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math


class GoToGoal(object):
    x0, y0, yaw0= 0, 0, 0
    self.goal_des=[0, 0, 0]

    def __init__(self, nodename):
        heading_threshold=3*math.pi/180

        rospy.init_node('go_to_goal')

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/target_goal", PoseStamped, self.goal_callback , queue_size=10)

        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            #if direction not similar, rotate
            if abs(self.yaw0-self.yaw_des)>heading_threshold:
                self.rotate(self.goal_des[2])
            else:
                #else translate to goal    
                self.translate(self.goal[0], self.goal[1])

            r.sleep()

    def goal_callback(self, msg):

        #store target goal as private variable
        _, _, yaw_des = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.goal_des=[msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_des]


    def rotate(self, angle):
        msg=Twist()
        angular_vel=200 #max is 660

        ang_error=math.atan2(math.sin(angle-self.yaw0), math.cos(angle-self.yaw0))

        if ang_error<math.pi:
            msg.angular.z=1024+angular_vel
        else:
            msg.angular.z=1024-angular_vel

        self.cmd_vel_pub.publish(msg)
        

    def translate(self, x_target, y_target):
        msg=Twist()
        vel=200
        distance_threshold=0.1

        x_error=(x_target-self.x0)*math.cos(self.yaw0)+(y_target-self.y0)*math.sin(self.yaw0)
        y_error=-(x_target-self.x0)*math.sin(self.yaw0)+(y_target-self.y0)*math.cos(self.yaw0)

        if abs(x_error)>distance_threshold:
            if x_error>0:
                msg.linear.x=1024+vel
            else:
                msg.linear.x=1024-vel

        if abs(y_error)>distance_threshold:
            if y_error>0:
                msg.linear.y=1024+vel
            else:
                msg.linear.y=1024-vel

        self.cmd_vel_pub.publish(msg)


    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
        

if __name__ == '__main__':
    try:
        GoToGoal(nodename="go_to_goal")
    except rospy.ROSInterruptException:
        rospy.loginfo("Go to goal finished.")

if __name__ == '__main__':


