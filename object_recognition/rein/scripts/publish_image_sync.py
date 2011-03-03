#!/usr/bin/env python

PACKAGE='rein'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rein.msg import MaskArray, DetectionArray
import message_filters
import cv
import os
import sys


import pdb




class ImagePublisherSync:

    def __init__(self, imagename, sync):
        rospy.init_node('image_pub_sync', anonymous=True)
        self.pub = rospy.Publisher('image', Image)
        if sync:
            self.sub = rospy.Subscriber('detections', DetectionArray, self.callback)

        self.bridge = CvBridge()

        cv_image = cv.LoadImage(imagename)
        self.image = self.bridge.cv_to_imgmsg(cv_image)
        self.image.header.frame_id = 'custom_image'
        if sync:
            self.publish()
        else:
            self.loop_publish()


    def loop_publish(self):
        r = rospy.Rate(2) 
        while not rospy.is_shutdown():
            self.publish()
            r.sleep()

    def publish(self):
        rospy.loginfo("Publishing image")
        self.image.header.stamp = rospy.rostime.get_rostime()
        self.stamp = self.image.header.stamp
        self.pub.publish(self.image)


    def callback(self,detections):
        if detections.header.stamp==self.stamp:
            rospy.loginfo("Publishing image")
            self.publish()
        else:
            rospy.logwarn("Received out of order message")



if __name__ == "__main__":

    argv = rospy.myargv()
    if len(argv)<2:
        rospy.logerr("Usage: %s [-s] image_name"%argv[0])
        sys.exit(1)

    sync = False
    if argv[1]=="-s":
        sync = True
    ips = ImagePublisherSync(argv[1], sync)
    rospy.spin()


