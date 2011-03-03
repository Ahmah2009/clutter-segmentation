#!/usr/bin/env python

PACKAGE='rein'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from sensor_msgs.msg import Image
from rein.msg import MaskArray

class NullMasksPublisher:

    def __init__(self, *args, **kwds):

        rospy.init_node('null_masks_publisher', anonymous=True)
        self.image_sub = rospy.Subscriber("image",Image, self.callback)
        self.masks_pub = rospy.Publisher("~masks",MaskArray)
        rospy.loginfo("Node init complete")

    def callback(self,image):
        masks = MaskArray()
        masks.header = image.header
        self.masks_pub.publish(masks)

if __name__ == "__main__":
    nmp = NullMasksPublisher()
    rospy.spin()
