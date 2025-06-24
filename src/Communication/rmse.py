#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import sewar
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class DynamicMapFusion:
    def __init__(self, rmse_thresh=8.0): # le seuil par défaut est de 8.0, comme dans l'article
        # seuil RMSD
        self.rmse_thresh = rmse_thresh

        # stocker la carte locale (mise à jour par gmapping)
        self.local_map = None # première carte locale reçue

        # cartes de fusion
        self.last_accepted = None       # dernière carte acceptée
        self.global_received = None     # fusion des reçues
        self.global_map = None          # fusion global_received + local

        # pubs / subs
        # todo : changer les noms pour éviter les conflits ofc
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.cb_local_map)
        self.sub_shared = rospy.Subscriber('/shared_map', OccupancyGrid, self.cb_shared_map)
        self.pub_shared = rospy.Publisher('/shared_map', OccupancyGrid, queue_size=1)

    def cb_local_map(self, msg):
        # on stocke la carte locale
        self.local_map = msg
        # si on a déjà une carte reçue, on peut recalculer global_map
        if self.global_received is not None:
            self.compute_global_map()
    def normalize_cell(cell):
        if cell == -1:
            return 0
        elif cell == 0:
            return 127
        elif cell >= 100:
            return 255
        else:
            pass # si jamais besoin de normalisation linéaire : return int(127 + (cell / 100.0) * (255 - 127)) # normalisation linéaire => note à moi même, 
        
    def cb_shared_map(self, msg):
        # on ignore ses propres publications
        if msg.header.frame_id == rospy.get_name():
            return

        # transforme en image (uint8 0..255)
        img = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        # normalisation des valeurs : -1 → 0, 0 → 127, 100 → 255
        vectorized_normalize = np.vectorize(normalize_cell)
        img = vectorized_normalize(img).astype(np.uint8)

        if self.last_accepted is None:
            # première carte reçue : on accepte sans condition
            self.global_received = img.copy()
            self.last_accepted = img.copy()
            rospy.loginfo("[Fusion] Acceptation initiale")
        else:
            # calcul RMSD
            rmsd = sewar.full_ref.rmsd(self.last_accepted, img)
            if rmsd > self.rmse_thresh:
                # on accepte
                rospy.loginfo(f"[Fusion] Acceptation (RMSD={rmsd:.2f})")
                # fusion reçues ← moy(@self.global_received, img)
                self.global_received = cv2.addWeighted(self.global_received, 0.5, img, 0.5, 0) # on n'a pas de poids pour la dernière carte acceptée, on la considère comme une carte reçue
                self.last_accepted = img.copy()
            else:
                rospy.loginfo(f"[Fusion] Refus (RMSD={rmsd:.2f} ≤ {self.rmse_thresh})")
                return

        # dès qu’on a mis à jour global_received, on peut recalculer global_map et republier
        self.compute_global_map()
        self.publish_global_map()

    def compute_global_map(self):
        # fusion locale + reçue
        # transforme local_map en img locale
        lm = np.array(self.local_map.data, dtype=np.int8).reshape((self.local_map.info.height, self.local_map.info.width))
        lm = ((lm + 1) * 127).astype(np.uint8)
        # moyenne pondérée
        fused = cv2.addWeighted(self.global_received, 0.5, lm, 0.5, 0)
        # median filter pour réduire le bruit
        self.global_map = cv2.medianBlur(fused, 3)

    def publish_global_map(self):
        # reconvertir global_map (uint8) en OccupancyGrid
        h, w = self.global_map.shape
        data = (self.global_map.astype(np.float32) / 127.0 - 1.0).reshape(-1)
        # clamp à [-1,100]
        data = np.clip(data, -1.0, 1.0)
        grid = OccupancyGrid()
        grid.header = Header(stamp=rospy.Time.now(), frame_id=rospy.get_name())
        grid.info = self.local_map.info  # même info
        # reconvertir : [-1→-1, 0→0, +1→100]
        grid.data = [int(d * 100) if d>=0 else -1 for d in data]
        self.pub_shared.publish(grid)
        rospy.loginfo("[fusion] carte fusionnée publiée")

if __name__ == '__main__':
    rospy.init_node('dynamic_map_fusion')
    rmse_th = rospy.get_param('~rmse_threshold', 8.0)
    fuser = DynamicMapFusion(rmse_th)
    rospy.spin()
