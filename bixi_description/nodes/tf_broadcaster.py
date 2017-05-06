#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math



class TfBroadcaster(object):
    x0, y0, yaw0= 0, 0, 0
    box_length=0.2
    edges=[]
    clustered_edges=[]
    box=[]
    n_edge=30
    cam_angle=math.pi/2

    def __init__(self, nodename):
        rospy.init_node('tf_broadcaster')

        rospy.Subscriber("/edge", PoseStamped, self.edgeCallback, queue_size = 50)


        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, math.pi, math.pi),
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


    def localizerCallback(self, msg):
        #publish odometry
        

if __name__ == '__main__':
    try:
        TfBroadcaster(nodename="identify_box")
    except rospy.ROSInterruptException:
        rospy.loginfo("Boxes detection finished.")

if __name__ == '__main__':


