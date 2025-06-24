#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

def cb(msg):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "odom"      # doit être "odom"
    t.child_frame_id = "base_link"          # relie au vrai frame
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation    = msg.pose.pose.orientation
    br.sendTransform(t)

if __name__=='__main__':
    rospy.init_node('odom_to_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/rvr1/odom', Odometry, cb)
    rospy.spin()
