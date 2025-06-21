#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import logging
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String
from skimage.filters.rank import median
from skimage.io import imread
from sewar.full_ref import rmse
from skimage.morphology import disk
from nav_msgs.msg import Odometry


def merge_images(img1, img2, i):
#"""
#merge with transparency + median
#"""


    im1 = imread(img1, cv2.IMREAD_UNCHANGED)
    im2 = imread(img2, cv2.IMREAD_UNCHANGED)
    grayImage1_new = im1.reshape(384, 384, 1)
    grayImage2_new = im2.reshape(384, 384, 1)
    #image recue a une proportion qui diminue en fonction du nombre d'images mergées
    alpha = 1-i
    combined_image = np.zeros(grayImage1_new.shape, dtype=grayImage1_new.dtype)
    combined_image[:,:,:] = (alpha * grayImage1_new[:,:,:]) + ((1-alpha) * grayImage2_new[:,:,:])
    im_med = combined_image[:,:,0] 
    med = median(im_med, disk((1)))
    
    return med
    
    
       
def merge_map_rvr_locale_global_received(im_locale, im_global_received, i):

#objectif : créer la carte à publier aux autre robots
#		carte qui sera un mix entre les cartes reçues (déjà merge avec transparence) et la carte locale du robot

#merge_images + median on local map + rmse
#au lieu d'avoir global, on parlera plutôt de carte global_received et self_global
#le merge fournira une carte self_globale qui sera publiée    

    img_locale_rvr = imread(im_locale, cv2.IMREAD_UNCHANGED)    #median filter only on local image
    img_locale_rvr_median = median(img_locale_rvr, disk((2)))
    img_global_received = imread(im_global_received, cv2.IMREAD_UNCHANGED)
    distance = rmse(img_locale_rvr, img_global_received)

    #if distance > 1: #means that everything is accepted 
    grayImage1_new = img_locale_rvr.reshape(384, 384, 1)
    grayImage2_new = img_global_received.reshape(384, 384, 1)
    alpha = i
    combined_image = np.zeros(grayImage1_new.shape, dtype=grayImage1_new.dtype)
    combined_image[:,:,:] = (alpha * grayImage1_new[:,:,:]) + ((1-alpha) * grayImage2_new[:,:,:])
    im_med = combined_image[:,:,0] 
    merge = median(im_med, disk((1)))
        
    #else:
        #merge = img_locale_rvr_median
        
    return merge


class Images:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self._cv_bridge = CvBridge()

        self.data = None
        self.flag_global = None
        self.image_sent_frame_id = None
        self.cv_imagesent = None
        self.data = None
        self.map_stamp = None 
        #self.name = rospy.get_param("~robot_id")
        self.name="rvr2"
        print("Robot ID:", self.name)
        self.i = 0
        self.m = 0
        self.u0 = 0
        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        
        
        name_list=["rvr0", "rvr1", "rvr2", "rvr3", "rvr4"]
        name_list.remove(self.name)
        rvrA = name_list[0]
        rvrB = name_list[1]
        rvrC = name_list[2]
        rvrD = name_list[3]
                
        self.global_Publisher = rospy.Publisher(self.name + "/global", Image, queue_size=10)
      
        rospy.Subscriber("map_sent", Image, self.map_sent_callback, queue_size=100, buff_size=200000*10) 
        rospy.Subscriber(self.name + "/odom", Odometry, self.odom_callback)
        rospy.Subscriber(str(rvrA) + "/odom", Odometry, self.odom_callback0)
        rospy.Subscriber(str(rvrB) + "/odom", Odometry, self.odom_callback1)
        rospy.Subscriber(str(rvrC) + "/odom", Odometry, self.odom_callback2)
        rospy.Subscriber(str(rvrD) + "/odom", Odometry, self.odom_callback3)

        self.get_pgm()
        
    def map_sent_callback(self, msg):
        try:
        
            name_list=["rvr0", "rvr1", "rvr2", "rvr3", "rvr4"]
            name_list.remove(self.name)
            rvrA = name_list[0]
            rvrB = name_list[1]
            rvrC = name_list[2]
            rvrD = name_list[3]
            
            self.m += 1
            print(self.name, "received", self.m, "images")
            
            #print("sent_id, sent_header", msg.header.frame_id, msg.header.stamp)
            self.image_sent_frame_id = msg.header.frame_id
            self.cv_imagesent = self._cv_bridge.imgmsg_to_cv2(msg, "mono8")
            self.map_stamp = msg.header.stamp
            #print('test', self.map_rvr)  OK 
            #print('test', self.rvr_x)    OK
            if self.image_sent_frame_id != self.name: 
                
                #compute distance
                if self.image_sent_frame_id == str(rvrA):
                    #print('distA')
                    p=[self.rvr_x, self.rvr_y]
                    q=[self.rvr_x0, self.rvr_y0]
                    #print('p', p)
                    #print('q', q)
                    dist = math.dist(p, q)
                    #print('dist', dist)
                    self.u0 += 1
                    print(msg.header.stamp, self.name, "received", self.u0, "images from 0 ", self.image_sent_frame_id, "last distance is", dist)
                    
                elif self.image_sent_frame_id == str(rvrB):
                    #print('distA')
                    p=[self.rvr_x, self.rvr_y]
                    q=[self.rvr_x1, self.rvr_y1]
                    #print('p', p)
                    #print('q', q)
                    dist = math.dist(p, q)
                    #print('dist', dist)
                    self.u1 += 1
                    print(msg.header.stamp, self.name, "received", self.u1, "images from 1 ", self.image_sent_frame_id, "last distance is", dist)
                    
                elif self.image_sent_frame_id == str(rvrC):
                    #print('distA')
                    p=[self.rvr_x, self.rvr_y]
                    q=[self.rvr_x2, self.rvr_y2]
                    #print('p', p)
                    #print('q', q)
                    dist = math.dist(p, q)
                    #print('dist', dist)
                    self.u2 += 1
                    print(msg.header.stamp, self.name, "received", self.u2, "images from 2 ", self.image_sent_frame_id, "last distance is", dist)
                    
                else:
                    #print('distB')
                    p=[self.rvr_x, self.rvr_y]
                    #print('p',p)
                    q=[self.rvr_x3, self.rvr_y3]
                    #print('q',q)
                    dist = math.dist(p, q)
                    #print('dist',dist)
                    self.u3 += 1
                    print(msg.header.stamp, self.name, "received", self.u3, "images from 3 ", self.image_sent_frame_id, "last distance is", dist)

                if dist < 1 :
                    self.i += 1
                    print(msg.header.stamp, "rvr", self.name, "accepted with a distance smaller than 1 a total of", self.i, "images")
                    try:
                        received_previously = cv2.imread(self.map_received_rvr, cv2.IMREAD_UNCHANGED)
                        #rmse_merge = rmse(self.cv_imagesent, received_previously)
                        if received_previously is not None :  # Check if image was received previously 
                            cv2.imwrite(self.map_cv, self.cv_imagesent)
                            
                            #essayer de bouger le imwrite que si image accepter
                            #calculer in second rmse avec le map_cv qui correspond maintenant à la dernière image reçue !!!!!
                            #permet de diminuer l'overconfidence
                            #pas vraiment, met surtout une image de plus 
                            
                            
                            rmse_merge = rmse(self.cv_imagesent, received_previously)
                            if rmse_merge > 4:  #check if new image is far enough from what the rvr already have before integrating it 
                                print(msg.header.stamp, self.name, "received an image from", self.image_sent_frame_id, "with a distance of ", dist, "and an rmse greater than 8 from the previous ones", rmse_merge)
                                try:    
                                    map_received_merge = merge_images(self.map_cv, self.map_received_rvr, 0.5)
                                    cv2.imwrite(self.map_received_rvr, map_received_merge)
                                except IOError as exc:
                                    print(exc) 
                            else:                                #discard new received image because too similar from the images already received 
                                print(msg.header.stamp, self.name, "received an image from", self.image_sent_frame_id, "with a distance of ", dist, "and an rmse smaller than 8 from the previous ones", rmse_merge)
                        else:
                            cv2.imwrite(self.map_received_rvr, self.cv_imagesent) 
                            print(msg.header.stamp, self.name, "received an image from", self.image_sent_frame_id, "with a distance of ", dist, "this is the first image received")
                    except IOError as exc:
                            print(exc)    
        except CvBridgeError as exc:
            print(exc)
            
       
    def odom_callback(self, data):
        try:
            self.rvr_x = data.pose.pose.position.x
            self.rvr_y = data.pose.pose.position.y
        except IOError as exc:
            print(exc)     
            
    def odom_callback0(self, data):
        try:
            self.rvr_x0 = data.pose.pose.position.x
            self.rvr_y0 = data.pose.pose.position.y
        except IOError as exc:
            print(exc)
            
    def odom_callback1(self, data):
        try:
            self.rvr_x1 = data.pose.pose.position.x
            self.rvr_y1 = data.pose.pose.position.y
        except IOError as exc:
            print(exc)
            
    def odom_callback2(self, data):
        try:
            self.rvr_x2 = data.pose.pose.position.x
            self.rvr_y2 = data.pose.pose.position.y
        except IOError as exc:
            print(exc)
            
    def odom_callback3(self, data):
        try:
            self.rvr_x3 = data.pose.pose.position.x
            self.rvr_y3 = data.pose.pose.position.y
        except IOError as exc:
            print(exc)
            
            
    def get_pgm(self):
        try:
            a = "/home/sphero-rvr/rvr_ros/map_capture/map_"
            d = "/home/sphero-rvr/rvr_ros/map_capture/map_globale_"
            e = "/home/sphero-rvr/rvr_ros/map_capture/map_received_to_"
            f = "/home/sphero-rvr/rvr_ros/map_capture/map_cv_of_"
            g = "/home/sphero-rvr/rvr_ros/map_capture/map_msg_of_"
            b = self.name
            c = ".pgm"
            self.map_rvr = a+b+c
            self.map_globale_rvr = d+b+c
            self.map_received_rvr = e+b+c
            self.map_cv = f+b+c
            self.map_msg = g+b+c
        except IOError as exc:
            print(exc)
            
              
    def local_global(self):
        while not rospy.is_shutdown():
            try:
                self.data = rospy.wait_for_message(self.name + "/flag", Float32, timeout=5)
                #print("robot_id", self.name)
            except rospy.ROSException as exc:
                print("waiting for argos")
                rospy.sleep(5)
            while self.data is not None :         
                try:
                    map_received_rvr = cv2.imread(self.map_received_rvr, cv2.IMREAD_UNCHANGED)
                    if map_received_rvr is not None:  # Check if image was read successfully
                        #  si au moins une image a été recue d'un autre robot
                        # envoyer le merge
                        #merge entre les maps recues déjà merge et la carte locale 
                        try:
                            map_globale_rvr = merge_map_rvr_locale_global_received(self.map_rvr, self.map_received_rvr, 0.5) 
                            cv2.imwrite(self.map_msg, map_globale_rvr) 
                            map_msg = cv2.imread(self.map_msg, cv2.IMREAD_UNCHANGED)
                            #creation du message 
                            rvr_msg = self._cv_bridge.cv2_to_imgmsg(map_msg, "mono8")
                            rvr_msg.header.stamp = rospy.Time.now()
                            rvr_msg.header.frame_id = self.name
                            self.global_Publisher.publish(rvr_msg) 
                        except IOError as exc:
                            print(exc)
                        
                        #si aucune image n'a été reçue d'un autre robot
                        #envoyer la carte locale 
                    else:
                        try:
                            local_rvr = cv2.imread(self.map_rvr, cv2.IMREAD_UNCHANGED)
                            if local_rvr is not None:
                                rvr_msg = self._cv_bridge.cv2_to_imgmsg(local_rvr, "mono8")
                                rvr_msg.header.stamp = rospy.Time.now()
                                rvr_msg.header.frame_id = self.name
                                self.global_Publisher.publish(rvr_msg)
                        except IOError as exc:
                            print(exc)
    
                except IOError as exc:
                    print(exc)

                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("images", anonymous=True)
    Images().local_global()
