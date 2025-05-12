#!/usr/bin/env python3
import rospy, math, time
from sensor_msgs.msg import LaserScan
from sphero_sdk import SpheroRvrObserver
from driver_logger import DriverLogger

"""
LidarDrivingTest - Basic control of an RVR robot using a LIDAR

This ROS script allows a Sphero RVR robot to automatically move forward in a straight line
as long as no obstacle is detected within a frontal cone defined by an angle of ±30°.
It relies on data from the LIDAR sensor (ROS topic of type `sensor_msgs/LaserScan`) and
decides, at regular intervals, whether to continue moving forward or stop.

How it works:
- The robot wakes up and initializes its orientation (yaw reset).
- The LIDAR is monitored via a ROS subscriber that continuously computes the minimum distance
  in the specified frontal cone.
- Every 0.25 seconds, a timer triggers a decision function:
    - If an obstacle is detected within 0.4 m, the robot stops.
    - Otherwise, it moves forward at 0.2 m/s.
- Logs are emitted via a parent class `DriverLogger` to facilitate tracking/debugging.

Notes:
- The LIDAR topic (`LIDAR_TOPIC`) should be adapted according to the actual setup (e.g., `/rvr2/scan`).
- The script is designed for ROS1 and uses `rospy`.
- Requires the `sphero_sdk` library and a driver for the connected RVR.

Author: Salman Houdaibi
"""


class LidarDrivingTest(DriverLogger):
    CALLBACK_INTERVAL = 0.25
    LIDAR_TOPIC      = '/rvr2/scan' # to change following your setup
    SEUIL_DISTANCE   = 0.4 # m
    VITESSE_AVANCE   = 0.2 # m/s
    HALF_ANGLE_DEG   = 30 # frontal half-angle
    HALF_ANGLE_RAD   = math.radians(HALF_ANGLE_DEG)

    def __init__(self):
        rospy.init_node('lidar_driving_test_30deg', anonymous=False)
        self.min_front = float('inf')
        # subscribe to the LIDAR
        rospy.Subscriber(self.LIDAR_TOPIC, LaserScan, self.lidar_cb, queue_size=1)
        # RVR initialization
        self.rvr = SpheroRvrObserver()
        self.log("Waking up RVR…") # usual log
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()
        # decision timer
        rospy.Timer(rospy.Duration(self.CALLBACK_INTERVAL), self.timer_cb)
        rospy.spin()

    def lidar_cb(self, msg: LaserScan):
        """
        For each ray i:
          angle_i = angle_min + i * angle_increment
          if |normalize(angle_i)| <= HALF_ANGLE_RAD, it is considered
        """
        amin = msg.angle_min
        ainc = msg.angle_increment
        dmin = float('inf')
        for i, d in enumerate(msg.ranges):
            # filter on validity
            if d < 0.05 or math.isinf(d) or math.isnan(d):
                continue
            # compute angle in [-pi, pi]
            ang = amin + i * ainc
            ang = math.atan2(math.sin(ang), math.cos(ang))
            # if within front ± HALF_ANGLE
            # The front is considered centered around ±π radians (i.e. 180°)
            front_angle = math.pi  # or -math.pi
            # angular delta between the ray and the virtual "front"
            delta = math.atan2(math.sin(ang - front_angle), math.cos(ang - front_angle))
            if abs(delta) <= self.HALF_ANGLE_RAD and d < dmin:
                dmin = d
        self.min_front = dmin
        # throttled log for debugging
        rospy.loginfo_throttle(
            1.0, f"[LIDAR] min_front = {self.min_front:.2f} m (threshold {self.SEUIL_DISTANCE} m)"
        )

    def timer_cb(self, event):
        # decision every CALLBACK_INTERVAL sec
        if self.min_front < self.SEUIL_DISTANCE:
            self.log(f"Obstacle at {self.min_front:.2f} m → stopping")
            v = 0.0
        else:
            self.log("Front clear → moving forward")
            v = self.VITESSE_AVANCE

        # send command directly
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
