#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 15:41:54 2021

@author: robot
"""
from __future__ import print_function

#import roslib
#roslib.load_manifest('beginner_tutorials')
import message_filters
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import pickle
import math

class Camera_Lidar_Calibration:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_calibration",Image)
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/camera/color/image_raw",Image)
    self.scan_sub = message_filters.Subscriber("/scan",LaserScan)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.scan_sub], 10, 0.1, allow_headerless=True)
    self.ts.registerCallback(self.callback)
    with open('/home/robot/calibration/src/calibration/scripts/camera_lidar_calibrationq.p', "rb") as file:
        self.rvec = pickle.load(file)
        self.mtx = pickle.load(file)
        self.tvec = pickle.load(file)
    
  def callback(self, image, laser_scan):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
      lidar_data = laser_scan.ranges[278:453]
      
    except CvBridgeError as e:
      print(e)
    
    length = len(lidar_data)
    print(length)
    lidar_obj = []
    lidar_round = [0 for i in range(length)]
    
    for i in range(length):

        if lidar_data[i] == float("inf"):
            lidar_round[i] = 4000
            
        else:
            lidar = round(lidar_data[i]*100,3)
            lidar_round[i] = lidar
    
    theta = 137.5
    
    for i in lidar_round:
        lidar_obj_x = round(i*math.cos(theta*math.pi/180), 2)
        lidar_obj_y = round(i*math.sin(theta*math.pi/180), 2)
        lidar_obj_z = round(0, 3)
        lidar_obj_point = [lidar_obj_x, lidar_obj_y, lidar_obj_z]     
        lidar_obj.append(lidar_obj_point)
        theta = theta + 0.5
    
    lidar_objs = np.array(lidar_obj, dtype = "double")    
    lidpoint, _ = cv2.projectPoints(lidar_objs, self.rvec, self.tvec, self.mtx, np.zeros(5))

    for i in range(0, len(lidpoint)):
        x = int(lidpoint[i][0][0])
        y = int(lidpoint[i][0][1])
        distance = lidar_round[i]
        if distance < 150:
            color = (0,0,255)
        elif distance < 300:
            color = (255, 0, 0)
        else:
            color = (0, 255, 0)
        cv2.circle(cv_image, (x, y), 1, color, 3)
    
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
        
def main(args):
  print(cv2.__version__)
  rospy.init_node('CL_Calibration', anonymous=True)  
  CLC = Camera_Lidar_Calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    

#import message_filters
#from std_msgs.msg import Int32, Float32
#
#def callback(mode, penalty):
#  # The callback processing the pairs of numbers that arrived at approximately the same time
#
#mode_sub = message_filters.Subscriber('mode', Int32)
#penalty_sub = message_filters.Subscriber('penalty', Float32)
#
#ts = message_filters.ApproximateTimeSynchronizer([mode_sub, penalty_sub], 10, 0.1, allow_headerless=True)
#ts.registerCallback(callback)
#rospy.spin()
