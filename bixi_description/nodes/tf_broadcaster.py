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

        rospy.Subscriber("/localizer", Pose, self.edgeCallback, queue_size = 50)
        self.odom_pub=rospy.Publisher("/odometry", Odometry, queue_size=10)

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()
            #laser tf
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, math.pi, math.pi),
                             rospy.Time.now(),
                             "laser",
                             "base_link")
            #localizer tf
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, math.pi/2),
                             rospy.Time.now(),1
                             "encoder",
                             "base_link")
            

            #will be given by rplidar slam gmapping
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "odom",
                             "map")

            current_time = rospy.Time.now()
            odom.header.stamp = current_time
            odom_pub.publish(odom)
            
            r.sleep()


    def encoderCallback(self, msg):
        #broadcast odom tf
        #center-encoder
        del_x=0.2
        del_y=0.5
        #odom frame given by localizer, perform tranform first
        yaw_c=msg.yaw+math.pi/2
        x_c=msg.x+del_x*math.cos(yaw_c)-del_y*math.sin(yaw_c)
        y_c=msg.y+del_x*math.sin(yaw_c)+del_y*math.cos(yaw_c)

        br = tf.TransformBroadcaster()
        br.sendTransform((x_c, y_c, 0),
                         tf.transformations.quaternion_from_euler(0, 0, yaw_c),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        #publish odometry
        odom=Odometry()
        odom.pose.pose.position.x=x_c
        odom.pose.pose.position.y=y_c
        odom.pose.pose.orientation=tf.transformations.quaternion_from_euler(0, 0, yaw_c)

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        TfBroadcaster(nodename="tf")
    except rospy.ROSInterruptException:
        rospy.loginfo("tf broadcaster exit.")

if __name__ == '__main__':


