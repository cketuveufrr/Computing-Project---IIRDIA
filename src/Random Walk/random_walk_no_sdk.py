#!/usr/bin/env python3

import rospy, math, time, random
from sensor_msgs.msg import LaserScan
from driver_logger import DriverLogger
from geometry_msgs.msg import Twist

class BallisticRandomWalk(DriverLogger):
    LIDAR_TOPIC        = '/rvr1/scan'
    OBSTACLE_THRESHOLD = 0.4       # m
    FORWARD_SPEED      = 0.3       # m/s
    TURN_SPEED         = 2.0
    HALF_ANGLE_DEG     = 30
    HALF_ANGLE_RAD     = math.radians(HALF_ANGLE_DEG)
    LOOP_HZ            = 20        # décision à 20 Hz

    def __init__(self):
        rospy.init_node('ballistic_random_walk', anonymous=False)

        self.state    = 'FORWARD'    # ou 'TURN'
        self.turn_end = 0.0
        self.turn_dir = 1
        self.min_front= float('inf')

        # Subscriber LIDAR
        rospy.Subscriber(self.LIDAR_TOPIC, LaserScan, self.lidar_cb, queue_size=1)
        # Publisher cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.main_loop()

    def lidar_cb(self, msg: LaserScan):
        """Mise à jour immédiate de la plus petite distance frontale."""
        dmin = float('inf')
        amin = msg.angle_min
        ainc = msg.angle_increment

        for i, d in enumerate(msg.ranges):
            if not (0.05 < d < float('inf')):
                continue
            ang = amin + i * ainc
            ang = math.atan2(math.sin(ang), math.cos(ang))
            delta = math.atan2(math.sin(ang - math.pi), math.cos(ang - math.pi))
            if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                dmin = d

        # si obstacle critique, on bascule immédiatement
        if self.state == 'FORWARD' and dmin < self.OBSTACLE_THRESHOLD:
            # arrêt d’urgence et début de rotation
            self.publish_cmd(0.0, 0.0)
            dur = random.uniform(0.5, 2.0)
            self.turn_dir = random.choice([-1, 1])
            self.turn_end = time.time() + dur
            self.state    = 'TURN'
            self.log(f"EMERGENCY: obstacle à {dmin:.2f} m → tourner {'droite' if self.turn_dir>0 else 'gauche'} {dur:.2f}s")

        self.min_front = dmin

    def publish_cmd(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def main_loop(self):
        rate = rospy.Rate(self.LOOP_HZ)
        while not rospy.is_shutdown():
            now = time.time()

            if self.state == 'FORWARD':
                # si lidar_cb n'a pas déclenché d'urgence, avancer
                if self.min_front >= self.OBSTACLE_THRESHOLD:
                    self.publish_cmd(self.FORWARD_SPEED, 0.0)

            else:  # état TURN
                if now < self.turn_end:
                    self.publish_cmd(0.0, self.turn_dir * self.TURN_SPEED)
                else:
                    # fin de rotation → retour en FORWARD
                    self.state = 'FORWARD'
                    self.log("Rotation terminée → reprise de l'avance")
                    # publie plusieurs fois pour fiabiliser la commande
                    for _ in range(3):
                        self.publish_cmd(self.FORWARD_SPEED, 0.0)
                        rospy.sleep(1.0 / self.LOOP_HZ)

            rate.sleep()

if __name__ == "__main__":
    try:
        BallisticRandomWalk()
    except rospy.ROSInterruptException:
        pass
