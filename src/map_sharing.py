#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from skimage.filters import median
from skimage.morphology import disk
from sewar.full_ref import rmse
import tf2_ros

class MapSharingNode:
    def __init__(self):
        # Initialisation du nœud
        rospy.init_node('map_sharing_node') # renommer le nœud pour éviter les conflits

        # Paramètres configurables
        self.robot_id = rospy.get_param('~robot_id') #rvr1 par ex 
        self.distance_threshold = rospy.get_param('~distance_thresh', 1.0)  # m
        self.rmse_threshold     = rospy.get_param('~rmse_thresh', 8.0)      # seuil RMSD
        local_map_path = rospy.get_param('~local_map_path')           # chemin vers map.pgm

        # Chargement de la carte locale (image PGM, niveaux de gris)
        self.local_map = cv2.imread(local_map_path, cv2.IMREAD_GRAYSCALE)
        if self.local_map is None:
            rospy.logerr("Impossible de charger la carte locale : %s", local_map_path)
            rospy.signal_shutdown("erreur de chargement carte")

        # Etats internes
        self.last_accepted    = None    # dernière image acceptée
        self.global_received  = None    # fusion cumulée des images reçues

        # Pour conversion ROS <-> OpenCV
        self.bridge = CvBridge()

        # Pour récupérer la position des autres robots
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Abonnements et publications
        self.sub_img = rospy.Subscriber('/common_map', Image, self.image_callback, queue_size=5)
        self.pub_map  = rospy.Publisher('/global_map', Image, queue_size=1)

        rospy.loginfo("Node de partage d’images initialisé pour %s", self.robot_id)
        rospy.spin()

    def image_callback(self, msg):
        src_id = msg.header.frame_id
        # 1) Ignorer son propre message
        if src_id == self.robot_id:
            return

        # 2) Vérifier la distance via TF
        try:
            tf = self.tf_buffer.lookup_transform(self.robot_id,
                                                 src_id,
                                                 rospy.Time(0),
                                                 rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Pas de transform entre %s et %s", self.robot_id, src_id)
            return
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        dist = np.hypot(dx, dy)
        if dist > self.distance_threshold:
            # trop loin : on jette
            return

        # 3) Conversion en image OpenCV niveaux de gris
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # 4) Première image acceptée sans condition
        if self.last_accepted is None:
            self._accept_image(img)
            return

        # 5) Calcul du RMSD entre dernière acceptée et candidate
        score = rmse(self.last_accepted.astype(np.float32),
                     img.astype(np.float32))
        if score <= self.rmse_threshold:
            self._accept_image(img)
        else:
            rospy.loginfo("Image de %s rejetée (RMSD=%.2f)", src_id, score)

    def _accept_image(self, img):
        # Mettre à jour la dernière image acceptée
        self.last_accepted = img.copy()

        # Fusionner dans la carte reçue cumulée
        if self.global_received is None:
            self.global_received = img.copy()
        else:
            self.global_received = self._merge(self.global_received, img)

        # Fusionner carte reçue + carte locale pour produire la carte globale
        global_map = self._merge(self.global_received, self.local_map)

        # Publier la carte globale
        out_msg = self.bridge.cv2_to_imgmsg(global_map, encoding='mono8')
        out_msg.header.stamp = rospy.Time.now()
        out_msg.header.frame_id = self.robot_id
        self.pub_map.publish(out_msg)
        rospy.loginfo("Publiée carte globale mise à jour par %s", self.robot_id)

    def _merge(self, img1, img2):
        # 1) Redimensionner si nécessaire
        if img1.shape != img2.shape:
            img2 = cv2.resize(img2,
                              (img1.shape[1], img1.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        # 2) Mélange pondéré à 50/50
        blended = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)
        # 3) Filtre médian (élément structurant disque rayon=1)
        filtered = median(blended, disk(1))
        # 4) Assurer type uint8
        return (filtered.astype(np.uint8) if filtered.dtype != np.uint8
                else filtered)

if __name__ == '__main__':
    try:
        MapSharingNode()
    except rospy.ROSInterruptException:
        pass
