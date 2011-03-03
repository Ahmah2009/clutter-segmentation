#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('pano_py')
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import shutil
import cv

def grab(file, folder):
    bridge = CvBridge()
    if file.endswith('.bag'):
       bag = rosbag.Bag(file,'r')
       index = 1
       for topic, msg, t in bag.read_messages(topics=['/camera/rgb/image_color']):
         try:
            cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
            image_name = os.path.join(folder, str(index) + '.png')
            index = index + 1
            cv.SaveImage(image_name, cv_image)
            print "saved %s" % image_name
         except CvBridgeError, e:
            print e

