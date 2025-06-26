#!/usr/bin/env python3

import rospy, math, time, random
from sensor_msgs.msg import LaserScan
from driver_logger import DriverLogger
from geometry_msgs.msg import Twist

class BallisticRandomWalk(DriverLogger):
    OBSTACLE_THRESHOLD = 0.4       # m
    FORWARD_SPEED      = 0.2       # m/s
    TURN_SPEED         = 2.0
    HALF_ANGLE_DEG     = 60
    HALF_ANGLE_RAD     = math.radians(HALF_ANGLE_DEG)

    FRONT_ANGLE_RAD    = 1.57       # rad : orientation réelle de l’avant (0 si front à 0 rad)

    LOOP_HZ            = 50        # décision à 50 Hz

    def __init__(self):
        rospy.init_node('ballistic_random_walk', anonymous=False)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.lidar_topic   = rospy.get_param("~lidar_topic", "/scan")
        self.state    = 'FORWARD'
        self.turn_end = 0.0
        self.turn_dir = 1
        self.min_front= float('inf')

        rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.main_loop()

    def lidar_cb(self, msg: LaserScan):
        """Mise à jour de la distance minimale dans le cône avant défini. """
        dmin = float('inf')
        amin = msg.angle_min
        ainc = msg.angle_increment

        for i, d in enumerate(msg.ranges):
            # filtre des mesures invalides
            if not (0.05 < d < float('inf')):
                continue

            # angle de ce rayon, normalisé entre -π et +π
            ang = amin + i * ainc
            ang = math.atan2(math.sin(ang), math.cos(ang))

            # écart angulaire par rapport à l’avant réel
            delta = math.atan2(
                math.sin(ang - self.FRONT_ANGLE_RAD),
                math.cos(ang - self.FRONT_ANGLE_RAD)
            )

            # si dans le cône frontal et plus proche
            if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                dmin = d

        # réaction immédiate en cas d’obstacle trop proche
        if self.state == 'FORWARD' and dmin < self.OBSTACLE_THRESHOLD:
            self.publish_cmd(0.0, 0.0)
            dur = random.uniform(0.5, 2.0)
            self.turn_dir = random.choice([-1, 1])
            self.turn_end = time.time() + dur
            self.state    = 'TURN'
            self.log(f"EMERGENCY: obstacle à {dmin:.2f} m → tourner "
                     f"{'droite' if self.turn_dir>0 else 'gauche'} pendant {dur:.2f}s")

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
                if self.min_front >= self.OBSTACLE_THRESHOLD:
                    self.publish_cmd(self.FORWARD_SPEED, 0.0)

            else:  # TURN
                if now < self.turn_end:
                    self.publish_cmd(0.0, self.turn_dir * self.TURN_SPEED)
                else:
                    self.state = 'FORWARD'
                    self.log("Rotation terminée → reprise de l'avance")
                    for _ in range(3):
                        self.publish_cmd(self.FORWARD_SPEED, 0.0)
                        rospy.sleep(1.0 / self.LOOP_HZ)

            rate.sleep()

if __name__ == "__main__":
    try:
        BallisticRandomWalk()
    except rospy.ROSInterruptException:
        pass
