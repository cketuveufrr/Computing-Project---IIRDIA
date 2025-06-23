#!/usr/bin/env python3
import rospy, math, time, random
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from sphero_sdk import SpheroRvrObserver
from driver_logger import DriverLogger

class BallisticRandomWalk(DriverLogger):
    CALLBACK_INTERVAL   = 0.2
    NAMESPACE           = "rvr1" #ici c'est l'ID du RVR, par défaut on commence à 1
    LIDAR_TOPIC         = f'/{NAMESPACE}/scan'
    FLAG_TOPIC          = f'/{NAMESPACE}/flag'
    WHEELS_TOPIC        = f'/{NAMESPACE}/wheels_speed'
    OBSTACLE_THRESHOLD  = 0.4
    FORWARD_SPEED       = 0.2
    HALF_ANGLE_RAD      = math.radians(30)

    def __init__(self):
        # init ROS
        rospy.init_node('ballistic_random_walk', anonymous=False)
        # publishers
        self.flag_pub   = rospy.Publisher(self.FLAG_TOPIC, Bool, queue_size=1)
        self.wheels_pub = rospy.Publisher(self.WHEELS_TOPIC, Float32MultiArray, queue_size=1)
        # état
        self.state      = 'FORWARD'
        self.turn_end   = 0.0
        self.turn_dir   = 1
        self.min_front  = float('inf')
        # RVR hardware (optionnel : tu peux garder ou commenter)
        self.rvr = SpheroRvrObserver()
        self.log("Réveil du RVR…")
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()
        # subscriber LIDAR
        rospy.Subscriber(self.LIDAR_TOPIC, LaserScan, self.lidar_cb, queue_size=1)
        # boucle de contrôle
        rospy.Timer(rospy.Duration(self.CALLBACK_INTERVAL), self.control_cb)
        rospy.spin()

    def lidar_cb(self, msg: LaserScan):
        dmin = float('inf')
        ang = msg.angle_min
        for d in msg.ranges:
            if 0.05 < d < float('inf'):
                # normalisation angulaire dans [-π,π]
                delta = math.atan2(math.sin(ang - math.pi),
                                   math.cos(ang - math.pi))
                if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                    dmin = d
            ang += msg.angle_increment
        self.min_front = dmin

    def control_cb(self, event):
        now = time.time()
        # on signale qu’on est « alive » / en exploration
        self.flag_pub.publish(Bool(data=True))

        if self.state == 'FORWARD':
            if self.min_front < self.OBSTACLE_THRESHOLD:
                # obstacle → on pivote aléatoirement
                dur = random.uniform(0.5, 2.0)
                self.turn_dir = random.choice([-1, 1])
                self.turn_end = now + dur
                self.state    = 'TURN'
                self.log(f"Obstacle à {self.min_front:.2f} m → pivot `{self.turn_dir}` pour {dur:.2f}s")
            else:
                # front libre → on avance
                self.publish_wheels(self.FORWARD_SPEED, self.FORWARD_SPEED)

        else:  # en train de tourner
            if now < self.turn_end:
                lv = -self.turn_dir * self.FORWARD_SPEED
                rv = +self.turn_dir * self.FORWARD_SPEED
                self.publish_wheels(lv, rv)
            else:
                # fin de rotation
                self.state = 'FORWARD'
                self.log("Rotation terminée → reprise de l'avance")
                self.publish_wheels(self.FORWARD_SPEED, self.FORWARD_SPEED)

    def publish_wheels(self, left, right):
        # publication ROS
        msg = Float32MultiArray(data=[left, right])
        self.wheels_pub.publish(msg)
        # pilotage direct du RVR (optionnel)
        self.rvr.drive_tank_si_units(
            left_velocity  = left,
            right_velocity = right,
            timeout        = self.CALLBACK_INTERVAL * 0.9
        )

if __name__ == "__main__":
    try:
        BallisticRandomWalk()
    except rospy.ROSInterruptException:
        pass
