#!/usr/bin/env python3
import rospy, math, time
from sensor_msgs.msg import LaserScan
from sphero_sdk import SpheroRvrObserver
from driver_logger import DriverLogger

class LidarDrivingTest(DriverLogger):
    CALLBACK_INTERVAL = 0.25    # secondes
    # **Mets ici ton topic exact** :
    LIDAR_TOPIC      = '/rvr2/scan'
    SEUIL_DISTANCE   = 0.4      # m
    VITESSE_AVANCE   = 0.2      # m/s
    HALF_ANGLE_DEG   = 30       # demi-ouverture frontale
    HALF_ANGLE_RAD   = math.radians(HALF_ANGLE_DEG)

    def __init__(self):
        rospy.init_node('lidar_driving_test', anonymous=False)
        # état courant de la distance min frontale
        self.min_front = float('inf')
        # abonne-toi au LIDAR
        rospy.Subscriber(self.LIDAR_TOPIC, LaserScan, self.lidar_cb, queue_size=1)
        # initialisation du RVR
        self.rvr = SpheroRvrObserver()
        self.log("Réveil du RVR…")
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()
        # timer de décision
        rospy.Timer(rospy.Duration(self.CALLBACK_INTERVAL), self.timer_cb)
        rospy.spin()

    def lidar_cb(self, msg: LaserScan):
        """
        Pour chaque rayon i :
          angle_i = angle_min + i * angle_increment
          si |normalize(angle_i)| <= HALF_ANGLE_RAD, on le considère
        """
        amin = msg.angle_min
        ainc = msg.angle_increment
        dmin = float('inf')
        for i, d in enumerate(msg.ranges):
            # filtre sur validité
            if d < 0.05 or math.isinf(d) or math.isnan(d):
                continue
            # calcule l'angle dans [-pi,pi]
            ang = amin + i * ainc
            ang = math.atan2(math.sin(ang), math.cos(ang))
            # si dans le front ± HALF_ANGLE
            # On considère l’avant comme centré autour de ±π radians (i.e. 180°)
        front_angle = math.pi  # ou -math.pi
        # delta angulaire entre le rayon et l’"avant" virtuel
        delta = math.atan2(math.sin(ang - front_angle), math.cos(ang - front_angle))
        if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                dmin = d
        self.min_front = dmin
        # log throttlé pour debug
        rospy.loginfo_throttle(
            1.0, f"[LIDAR] min_front = {self.min_front:.2f} m (seuil {self.SEUIL_DISTANCE} m)"
        )

    def timer_cb(self, event):
        # décision toutes les CALLBACK_INTERVAL sec
        if self.min_front < self.SEUIL_DISTANCE:
            self.log(f"Obstacle à {self.min_front:.2f} m → arrêt")
            v = 0.0
        else:
            self.log("Front dégagé → avance")
            v = self.VITESSE_AVANCE

        # envoi direct de la commande
        self.rvr.drive_tank_si_units(
            left_velocity  = v,
            right_velocity = v,
            timeout        = self.CALLBACK_INTERVAL * 0.9
        )


if __name__ == "__main__":
    try:
        LidarDrivingTest()
    except rospy.ROSInterruptException:
        pass
