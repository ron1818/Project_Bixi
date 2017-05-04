#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math

if __name__ == '__main__':
    rospy.init_node('lasertf_broadcaster')


    odom_pub = rospy.Publisher("odometry/filtered", Odometry, queue_size=10)


    odom = Odometry()

    odom.header.frame_id = "odom"

    q=Quaternion()

    q.x, q.y, q.w, q.z=tf.transformations.quaternion_from_euler(0, 0, 0)


    # set the position
    odom.pose.pose = Pose(Point(0, 0, 0), q)

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "imu_link",
                         "base_link")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "odom",
                         "map")


        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom_pub.publish(odom)
        
        r.sleep()


