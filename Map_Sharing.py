#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage
from skimage.filters import median
from skimage.morphology import disk
from sewar.full_ref import rmse
import tf2_ros

def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """Convertit un Image ROS mono8 en tableau NumPy (H×W, dtype=uint8)."""
    if msg.encoding != 'mono8':
        raise ValueError(f"Encoding inattendu : {msg.encoding}")
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    return arr.reshape((msg.height, msg.width))

def numpy_to_image_msg(img: np.ndarray, frame_id: str) -> Image:
    """Emballe un np.ndarray mono8 en sensor_msgs/Image."""
    out = Image()
    out.header.stamp = rospy.Time.now()
    out.header.frame_id = frame_id
    out.height = img.shape[0]
    out.width  = img.shape[1]
    out.encoding     = 'mono8'
    out.is_bigendian = 0
    out.step         = img.shape[1]
    out.data         = img.flatten().tolist()
    return out

class MapSharingNode:
    def __init__(self):
        rospy.init_node('map_sharing_node')
        # Lecture des paramètres
        self.robot_id          = rospy.get_param('~robot_id')
        self.distance_threshold = rospy.get_param('~distance_thresh', 1.0)
        self.rmse_threshold     = rospy.get_param('~rmse_thresh', 8.0)
        local_map_path         = rospy.get_param('~local_map_path')

        # Chargement de la carte locale via Pillow
        try:
            pil = PILImage.open(local_map_path).convert('L')
            self.local_map = np.array(pil, dtype=np.uint8)
        except Exception as e:
            rospy.logerr("Impossible de charger la carte locale '%s' : %s",
                         local_map_path, e)
            rospy.signal_shutdown("erreur de chargement carte")

        # États internes
        self.last_accepted   = None
        self.global_received = None

        # TF pour la distance
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sub / Pub (sans slash, pour être dans le namespace)
        self.sub_img = rospy.Subscriber('common_map',
                                        Image,
                                        self.image_callback,
                                        queue_size=5)
        self.pub_map = rospy.Publisher('global_map',
                                       Image,
                                       queue_size=1)

        rospy.loginfo("MapSharingNode initialisé pour %s", self.robot_id)
        rospy.spin()

    def image_callback(self, msg: Image):
        src_id = msg.header.frame_id
        # 1) Ignorer son propre topic
        if src_id == self.robot_id:
            return

        # 2) Vérifier la distance via TF
        try:
            tf = self.tf_buffer.lookup_transform(self.robot_id,
                                                 src_id,
                                                 rospy.Time(0),
                                                 rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Pas de transform entre %s et %s", 
                          self.robot_id, src_id)
            return
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        if np.hypot(dx, dy) > self.distance_threshold:
            return

        # 3) Conversion ROS→NumPy
        img = image_msg_to_numpy(msg)

        # 4) Acceptation / rejet
        if self.last_accepted is None:
            self._accept_image(img)
            return

        score = rmse(self.last_accepted.astype(np.float32),
                     img.astype(np.float32))
        if score <= self.rmse_threshold:
            self._accept_image(img)
        else:
            rospy.loginfo("Image de %s rejetée (RMSD=%.2f)", src_id, score)

    def _accept_image(self, img: np.ndarray):
        # Met à jour dernier accepté
        self.last_accepted = img.copy()

        # Mise à jour cumulée
        if self.global_received is None:
            self.global_received = img.copy()
        else:
            self.global_received = self._merge(self.global_received, img)

        # Fusion cumulée + carte locale
        global_map = self._merge(self.global_received, self.local_map)

        # Publication
        out_msg = numpy_to_image_msg(global_map, self.robot_id)
        self.pub_map.publish(out_msg)
        rospy.loginfo("Publiée carte globale mise à jour par %s",
                      self.robot_id)

    def _merge(self, img1: np.ndarray, img2: np.ndarray) -> np.ndarray:
        # 1) Redimension si nécessaire (Pillow + nearest)
        if img1.shape != img2.shape:
            pil2 = PILImage.fromarray(img2)
            pil2 = pil2.resize((img1.shape[1], img1.shape[0]),
                               resample=PILImage.NEAREST)
            img2 = np.array(pil2, dtype=np.uint8)

        # 2) Blend 50/50 en numpy
        blended = (img1.astype(np.float32)*0.5 +
                   img2.astype(np.float32)*0.5)

        # 3) Filtre médian (skimage)
        filtered = median(blended, disk(1))

        # 4) Retour uint8
        arr = filtered.astype(np.uint8)
        return arr

if __name__ == '__main__':
    try:
        MapSharingNode()
    except rospy.ROSInterruptException:
        pass
