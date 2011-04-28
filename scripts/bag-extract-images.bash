#!/usr/bin/env python
import roslib; roslib.load_manifest('pano_py')
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import shutil
import cv

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print "Usage: bag-extract-images.py <bag-file> <target-dir>"
        sys.exit(1) 
    i = 0
    pano_links = []
    file = sys.argv[1]
    targetdir = sys.argv[2]
    bridge = CvBridge()
    bag = rosbag.Bag(file, 'r')
    i = 0
    for topic, msg, t in bag.read_messages(topics=['image']):
        try:
            print "***"
            cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
            image_name = os.path.join(targetdir, "image_%05d.jpg" % i)
            cv.SaveImage(image_name, cv_image)
            print "saved %s" % image_name
            i += 1
        except CvBridgeError, e:
            print e

