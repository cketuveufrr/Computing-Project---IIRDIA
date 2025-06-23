#!/usr/bin/env python3
"""
Nœud ROS qui sauvegarde périodiquement la carte (OccupancyGrid) en PGM + YAML.
"""

import os
import yaml
import rospy
import threading
import numpy as np
from nav_msgs.msg import OccupancyGrid
from datetime import datetime
import cv2

class MapSaver(object):
    def __init__(self):
        # Lecture des paramètres
        self.map_topic      = rospy.get_param('~map_topic', '/map')
        self.save_dir       = rospy.get_param('~save_directory', os.path.expanduser('~/.ros'))
        self.prefix         = rospy.get_param('~filename_prefix', 'map')
        self.interval       = rospy.get_param('~save_interval', 2.0)
        self.latest_map     = None
        self.map_lock       = threading.Lock()

        # Création du dossier s’il n’existe pas
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)

        # Souscription au topic map
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)

        # Timer périodique
        rospy.Timer(rospy.Duration(self.interval), self.timer_callback)
        rospy.loginfo(f"[map_saver] Listening on {self.map_topic}, saving every {self.interval}s into {self.save_dir}")

    def map_callback(self, msg: OccupancyGrid):
        with self.map_lock:
            self.latest_map = msg

    def timer_callback(self, event):
        with self.map_lock:
            if self.latest_map is None:
                return
            grid = self.latest_map

        # Conversion OccupancyGrid → image numpy (uint8)
        w = grid.info.width
        h = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        # Valeurs ROS: -1=unknown, 0=free, 100=occupied
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1]    = 205   # gris pour inconnu
        img[data == 0]     = 254   # blanc pour libre
        img[data >= 50]    = 0     # noir pour occupé

        # Génération des noms de fichiers
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pgm_path  = os.path.join(self.save_dir, f"{self.prefix}_{timestamp}.pgm")
        yaml_path = os.path.join(self.save_dir, f"{self.prefix}_{timestamp}.yaml")

        # Sauvegarde PGM
        cv2.imwrite(pgm_path, img)

        # Génération du YAML
        map_yaml = {
            'image': os.path.basename(pgm_path),
            'resolution': float(grid.info.resolution),
            'origin': [grid.info.origin.position.x,
                       grid.info.origin.position.y,
                       0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(map_yaml, f, default_flow_style=False)

        rospy.loginfo(f"[map_saver] Saved map → {pgm_path} + {yaml_path}")


if __name__ == '__main__':
    rospy.init_node('map_saver_python', anonymous=True)
    try:
        saver = MapSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
