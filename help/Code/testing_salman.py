#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from teraranger_array.msg import RangeArray
import math

current_ranges = []

def range_callback(msg):
    global current_ranges
    selected_indices = [2, 3, 4, 5]  # Capteurs 3 à 6
    current_ranges.clear()

    for i in selected_indices:
        if i < len(msg.ranges):
            dist = msg.ranges[i].range
            if math.isinf(dist):
                dist = 10.0  # Valeur "loin" si rien détecté
            current_ranges.append(dist)

if __name__ == '__main__':
    rospy.init_node('obstacle_stopper_selected_sensors')

    rospy.Subscriber('/ranges', RangeArray, range_callback)
    cmd_pub = rospy.Publisher('/rvr/wheels_speed', Float32MultiArray, queue_size=1)

    rate = rospy.Rate(50)  # 20 Hz

    while not rospy.is_shutdown():
        speeds = Float32MultiArray()

        if current_ranges:
            min_distance = min(current_ranges)
            rospy.loginfo_throttle(1.0, f"Min distance capteurs 3-6 : {min_distance:.2f} m")

            if min_distance < 0.3:
                rospy.loginfo("Obstacle devant → STOP")
                speeds.data = [0.0, 0.0]  # Stop
            else:
                speeds.data = [0.3, 0.3]  # Avance
        else:
            speeds.data = [0.0, 0.0]  # Sécurité : stop si pas de valeurs

        cmd_pub.publish(speeds)
        rate.sleep()
