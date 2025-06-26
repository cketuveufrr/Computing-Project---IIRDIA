#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfBroadcaster:
    def __init__(self):
        # Nom du node
        rospy.init_node('odom_to_tf_broadcaster', anonymous=False)

        # Paramètres (facultatifs)
        self.odom_topic   = rospy.get_param('~odom_topic', '/rvr1/odom')
        self.parent_frame = rospy.get_param('~parent_frame', 'odom')
        self.child_frame  = rospy.get_param('~child_frame', 'base_link')
        queue_size        = rospy.get_param('~queue_size', 20)

        # Création du broadcaster TF
        self.br = tf2_ros.TransformBroadcaster()

        # Abonnement
        rospy.Subscriber(self.odom_topic,
                         Odometry,
                         self.odom_callback,
                         queue_size=queue_size)

        rospy.loginfo(f"[odom_tf_broadcaster] Topic = {self.odom_topic}, TF = {self.parent_frame} → {self.child_frame}")

    def odom_callback(self, msg: Odometry):
        # Construction du message TransformStamped
        t = TransformStamped()

        # 1) Horodatage : si l’odométrie a un timestamp nul, on met “now”
        if msg.header.stamp == rospy.Time(0):
            t.header.stamp = rospy.Time.now()
        else:
            t.header.stamp = msg.header.stamp

        # 2) Cadres parent/enfant
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame

        # 3) Copie champ à champ de la position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        # on force z à 0 pour un plan 2D
        t.transform.translation.z = 0.0

        # 4) Copie de l’orientation
        t.transform.rotation = msg.pose.pose.orientation

        # 5) Diffusion de la transform
        self.br.sendTransform(t)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    broadcaster = OdomTfBroadcaster()
    broadcaster.run()
