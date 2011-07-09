#!/usr/bin/env python
import roslib
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import shutil
import cv

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print "usage: \033[1mbag-extract-images\033[0m [bag] [topic] [dir]"
        print
        print "Dumps images from a bag file to a directory."
        print
        print "bag      path to a bag file"
        print "topic    name of the image topic"
        print "dir      destination directory"
        sys.exit(1) 
    i = 0
    pano_links = []
    bagf = sys.argv[1]
    topicn = sys.argv[2]
    targetdir = sys.argv[3]
    if not os.path.exists(targetdir):
        print "Target directory does not exist."
        sys.exit(1) 
    bridge = CvBridge()
    bag = rosbag.Bag(bagf, 'r')
    i = 0
    for topic, msg, t in bag.read_messages(topics=[topicn]):
        try:
            print "***"
            cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
            image_name = os.path.join(targetdir, "image_%05d.jpg" % i)
            cv.SaveImage(image_name, cv_image)
            print "saved %s" % image_name
            i += 1
        except CvBridgeError, e:
            print e

