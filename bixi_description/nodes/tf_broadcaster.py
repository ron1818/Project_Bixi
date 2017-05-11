#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math



class TfBroadcaster(object):

    def __init__(self, nodename):
        rospy.init_node('tf_broadcaster')

        rospy.Subscriber("/cmd_vel", Twist, self.encoderCallback, queue_size = 50)
        self.odom_pub=rospy.Publisher("/odometry", Odometry, queue_size=10)

        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()
            #laser tf
            br.sendTransform((0.26, 0.25, 0),
                             tf.transformations.quaternion_from_euler(0, math.pi, math.pi),
                             rospy.Time.now(),
                             "laser",
                             "base_link")

            #localizer tf
            br.sendTransform((-0.05, -0.21, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "encoder",
                             "base_link")
            
            #will be given by rplidar slam gmapping
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "odom",
                             "map")
            
            r.sleep()


    def encoderCallback(self, msg):
        #broadcast odom tf
        #center-encoder
        del_x=0.05
        del_y=0.21
        #odom frame given by localizer, perform tranform first
        yaw_c=msg.angular.z*math.pi/180

        x_c=-msg.linear.y+del_x*math.cos(yaw_c)-del_y*math.sin(yaw_c)-del_x
        y_c=msg.linear.x+del_x*math.sin(yaw_c)+del_y*math.cos(yaw_c)-del_y
        #print(x_c, y_c)
        #print(msg.linear.x, msg.linear.y)

        br = tf.TransformBroadcaster()
        br.sendTransform((x_c, y_c, 0),
                         tf.transformations.quaternion_from_euler(0, 0, msg.angular.z*math.pi/180),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        #publish odometry
        odom=Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x=x_c
        odom.pose.pose.position.y=y_c
        q=Quaternion()
        q.x, q.y, q.z, q.w=tf.transformations.quaternion_from_euler(0, 0, msg.angular.z*math.pi/180)
        odom.pose.pose.orientation=q
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        TfBroadcaster(nodename="tf")
    except rospy.ROSInterruptException:
        rospy.loginfo("tf broadcaster exit.")

