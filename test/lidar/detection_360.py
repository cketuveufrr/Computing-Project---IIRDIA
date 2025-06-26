#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
import time

class LidarDistanceViewer:
    def __init__(self):
        rospy.init_node("lidar_distance_viewer")
        rospy.loginfo("Initialisation du Lidar Distance Viewer")

        self.last_display_time = time.time()
        self.display_interval = 2.0  # secondes entre chaque affichage

        self.scan_sub = rospy.Subscriber("/rvr2/scan", LaserScan, self.scan_callback)

        rospy.spin()

    def scan_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_display_time < self.display_interval:
            return  # on attend le prochain cycle pour afficher

        self.last_display_time = current_time

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        secteur_angle_deg = 30
        secteur_angle_rad = math.radians(secteur_angle_deg)

        output = "\n[Distances par secteur 30° - affichage toutes les 2s]\n"
        for i in range(12):
            angle_central = -math.pi + (i * secteur_angle_rad)
            index_center = int((angle_central - angle_min) / angle_increment)

            angle_range_indices = int(secteur_angle_rad / 2 / angle_increment)
            indices = range(index_center - angle_range_indices, index_center + angle_range_indices + 1)

            distances_valides = []
            for idx in indices:
                if 0 <= idx < len(ranges):
                    d = ranges[idx]
                    if not math.isinf(d) and not math.isnan(d) and d > 0.05:
                        distances_valides.append(d)

            if distances_valides:
                distance_min = min(distances_valides)
                output += "Secteur {:>3}°–{:>3}° : {:.2f} m\n".format(i*30, (i+1)*30, distance_min)
            else:
                output += "Secteur {:>3}°–{:>3}° : aucune donnée\n".format(i*30, (i+1)*30)

        rospy.loginfo(output)

if __name__ == "__main__":
    LidarDistanceViewer()
