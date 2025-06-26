#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos

class OdomImuCorrectorNode:
    def __init__(self):
        rospy.init_node('odom_imu_corrector', anonymous=True)

        # Topics & frames
        self.imu_topic      = rospy.get_param('~imu_topic',      '/rvr1/imu')
        self.odom_topic     = rospy.get_param('~odom_topic',     '/rvr1/odom')
        self.corrected_topic= rospy.get_param('~corrected_topic','/rvr1/odom_corrected')
        self.odom_frame     = rospy.get_param('~odom_frame',     'odom')
        self.base_frame     = rospy.get_param('~base_frame',     'base_link')
        self.corrected_imu_topic = rospy.get_param('~corrected_imu_topic', '/rvr1/imu_corrected')
        # Subscribers / Publishers
        self.imu_sub  = rospy.Subscriber(self.imu_topic, Imu,      self.imu_cb,  queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        
        self.odom_pub = rospy.Publisher(self.corrected_topic, Odometry, queue_size=10)
        self.imu_pub  = rospy.Publisher(self.corrected_imu_topic, Imu, queue_size=10)
        # TF broadcaster for odom->base_link
        self.tf_br = tf2_ros.TransformBroadcaster()

        # Stockage des valeurs initiales pour recentrage
        self.init_odom_x   = None
        self.init_odom_y   = None
        self.init_odom_yaw = None
        self.init_imu_yaw  = None

        # Dernières valeurs reçues
        self.last_imu_yaw = None

        rospy.loginfo("odom_imu_corrector started")
        rospy.spin()

    def imu_cb(self, msg: Imu):
        # republier l'IMU dans le bon frame
        msg.header.frame_id = self.base_frame
        self.imu_pub.publish(msg)

        # extraire yaw
        q = msg.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        # mémoriser la yaw initiale
        if self.init_imu_yaw is None:
            self.init_imu_yaw = yaw
            rospy.loginfo("Initial IMU yaw: %.3f rad", self.init_imu_yaw)
        self.last_imu_yaw = yaw

    def odom_cb(self, msg: Odometry):
        # on ne traite qu'après avoir un yaw IMU
        if self.last_imu_yaw is None:
            return

        # Pose odom brute
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oq = msg.pose.pose.orientation
        _, _, oyaw = euler_from_quaternion((oq.x, oq.y, oq.z, oq.w))

        # mémoriser l'origine odom une fois
        if self.init_odom_x is None:
            self.init_odom_x   = ox
            self.init_odom_y   = oy
            self.init_odom_yaw = oyaw
            rospy.loginfo("Initial odom origin: x=%.3f, y=%.3f, yaw=%.3f",
                          self.init_odom_x, self.init_odom_y, self.init_odom_yaw)

        # translation relative
        dx = ox - self.init_odom_x
        dy = oy - self.init_odom_y

        # orientation relative odom
        rel_odom_yaw = oyaw - self.init_odom_yaw

        # orientation relative IMU
        rel_imu_yaw = self.last_imu_yaw - self.init_imu_yaw

        # erreur d'orientation
        yaw_error = rel_imu_yaw - rel_odom_yaw

        # corriger la translation en pivotant de yaw_error
        cx =  cos(yaw_error)*dx - sin(yaw_error)*dy
        cy =  sin(yaw_error)*dx + cos(yaw_error)*dy

        # construire quaternion corrigé
        corr_q = quaternion_from_euler(0, 0, rel_imu_yaw)

        # publication de l'odom corrigé
        odom_out = Odometry()
        odom_out.header.stamp    = msg.header.stamp
        odom_out.header.frame_id = self.odom_frame
        odom_out.child_frame_id  = self.base_frame
        odom_out.pose.pose.position.x    = cx
        odom_out.pose.pose.position.y    = cy
        odom_out.pose.pose.position.z    = 0.0
        odom_out.pose.pose.orientation.x = corr_q[0]
        odom_out.pose.pose.orientation.y = corr_q[1]
        odom_out.pose.pose.orientation.z = corr_q[2]
        odom_out.pose.pose.orientation.w = corr_q[3]
        odom_out.twist = msg.twist  # on peut aussi filtrer ou recalculer
        self.odom_pub.publish(odom_out)

        # broadcast TF odom->base_link corrigé
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = cx
        t.transform.translation.y = cy
        t.transform.translation.z = 0.0
        t.transform.rotation.x = corr_q[0]
        t.transform.rotation.y = corr_q[1]
        t.transform.rotation.z = corr_q[2]
        t.transform.rotation.w = corr_q[3]
        self.tf_br.sendTransform(t)

        # logs pour debug
        rospy.loginfo(
            "Δodom=(%.3f, %.3f, %.3f) | ΔIMU=(%.3f) | err=%.3f | corr=(%.3f, %.3f, %.3f)",
            dx, dy, rel_odom_yaw,
            rel_imu_yaw,
            yaw_error,
            cx, cy, rel_imu_yaw
        )

if __name__ == '__main__':
    try:
        OdomImuCorrectorNode()
    except rospy.ROSInterruptException:
        pass
