#!/usr/bin/env python3

import rospy, math, time, random
from sensor_msgs.msg import LaserScan
from driver_logger import DriverLogger
from geometry_msgs.msg import Twist

class BallisticRandomWalk(DriverLogger): #héritage un peu hasardeux, mais pratique pour le logging
    CALLBACK_INTERVAL   = 0.2        # secondes entre décisions
    LIDAR_TOPIC         = '/rvr1/scan' # à changer bien évidemment si nécessaire
    OBSTACLE_THRESHOLD  = 0.4        # m : à partir de quelle distance on considère un obstacle
    FORWARD_SPEED       = 0.2        # m/s
    HALF_ANGLE_DEG     = 30         # demi-ouverture frontale pour la détection (±30°)
    HALF_ANGLE_RAD     = math.radians(HALF_ANGLE_DEG)

    def __init__(self):
        # init ROS
        rospy.init_node('ballistic_random_walk', anonymous=False)
        # état du robot
        self.state      = 'FORWARD'      # ou 'TURN'
        self.turn_end   = 0.0
        self.turn_dir   = 1
        self.min_front  = float('inf')
        # RVR
        # LIDAR
        rospy.Subscriber(self.LIDAR_TOPIC, LaserScan, self.lidar_cb, queue_size=1)
        # boucle périodique de contrôle
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(self.CALLBACK_INTERVAL), self.control_cb)
        rospy.spin()

    def lidar_cb(self, msg: LaserScan):
        """Repère la distance minimale dans la zone frontale."""
        dmin = float('inf')
        amin = msg.angle_min
        ainc = msg.angle_increment
        for i, d in enumerate(msg.ranges):
            if not (0.05 < d < float('inf')):
                continue
            ang = amin + i * ainc
            # normalisation dans [-π,π]
            ang = math.atan2(math.sin(ang), math.cos(ang))
            # calcul de l’écart angulaire vs l’avant (π)
            delta = math.atan2(math.sin(ang - math.pi),
                               math.cos(ang - math.pi))
            if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                dmin = d
        self.min_front = dmin
    def publish_cmd(self, linear: float, angular: float):
        """Publie une geometry_msgs/Twist pour le driver."""
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def control_cb(self, event):
        now = time.time()
        # --- État FORWARD ---
        if self.state == 'FORWARD':
            if self.min_front < self.OBSTACLE_THRESHOLD:
                # obstacle détecté → initie une rotation
                dur = random.uniform(0.5, 2.0)  # durée aléatoire de rotation
                self.turn_dir = random.choice([-1, 1])
                self.turn_end = now + dur
                self.state    = 'TURN'
                self.log(f"Obstacle à {self.min_front:.2f} m → pivot `{'+ ' if self.turn_dir>0 else '- '}{self.turn_dir}` pour {dur:.2f}s")
            else:
                # front libre → avance
                self.publish_cmd(linear=self.FORWARD_SPEED, angular=0.0)


        else:
            if now < self.turn_end:
                # en train de tourner
                rot_speed = self.turn_dir * 1.0
                self.publish_cmd(linear=0.0, angular=rot_speed)
            else:
                # fin de rotation → retour en FORWARD
                self.state = 'FORWARD'
                self.log("Rotation terminée → reprise de l'avance")
                self.publish_cmd(linear=self.FORWARD_SPEED, angular=0.0)
                self.publish_cmd(linear=self.FORWARD_SPEED, angular=0.0)




if __name__ == "__main__":
    try:
        BallisticRandomWalk()
    except rospy.ROSInterruptException:
        pass
