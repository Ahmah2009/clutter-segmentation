#!/usr/bin/env python

PACKAGE='rein'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rein.msg import MaskArray
import message_filters
import cv
import os


WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)


import wx
from wx_opencv import OpenCVImage 
from xml.dom.minidom import Document

import pdb

save_path = ''
class_name = ''


def check_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)



def create_annotation(imagename, image, mask_array):

    annotation=r'''<annotation>
	<folder>pan_tilt_objects</folder>
	<filename>%(filename)s</filename>
	<source>
		<database>Pan Tilt Objects</database>
	</source>
	<size>
		<width>%(width)d</width>
		<height>%(height)d</height>
		<depth>%(depth)d</depth>
	</size>
	<segmented>0</segmented>
	%(objects)s
</annotation>'''
    object =r'''<object>
		<name>%(name)s</name>
		<pose>Unspecified</pose>
		<truncated>0</truncated>
		<occluded>0</occluded>
		<bndbox>
			<xmin>%(xmin)g</xmin>
			<ymin>%(ymin)g</ymin>
			<xmax>%(xmax)g</xmax>
			<ymax>%(ymax)g</ymax>
		</bndbox>
		<difficult>0</difficult>
	</object>'''

    objects = ''

    for mask in mask_array.masks:
        objects += object%{'name':class_name,
                            'xmin':mask.roi.x,
                            'ymin':mask.roi.y,
                            'xmax':mask.roi.x+mask.roi.width,
                            'ymax':mask.roi.y+mask.roi.height}

    return annotation%{'filename':imagename, 
                        'width':image.width, 
                        'height':image.height,
                        'depth':image.channels,
                        'objects':objects}



def check_valid(image,mask_array):
    valid = True
    if len(mask_array.masks)==0:
        print "No rois"
        valid = False
    for mask in mask_array.masks:
        # check that the roi size is ok
        if mask.roi.width<50 or mask.roi.width<50:
            print "Wrong roi size"
            valid = False
        # check that the roi possition in image is ok
        if mask.roi.x<50 or mask.roi.y<50 or \
            image.width-mask.roi.x-mask.roi.width<50 or \
            image.height-mask.roi.y-mask.roi.height<50:
                print "Wrong roi position"
                valid = False
    return valid

def save_image_and_masks(name, image, mask_array):
    annotation_path = os.path.join(save_path, 'Annotations')
    check_dir(annotation_path)
    image_path = os.path.join(save_path, 'JPEGImages')
    check_dir(image_path)
    imageset_path = os.path.join(save_path, 'ImageSets','Main')
    check_dir(imageset_path)
    imageset = os.path.join(imageset_path, "trainval.txt")
    
    # write name to imageset
    f = open(imageset,'a+')
    print >>f, name
    f.close()

    #save image
    imagename = "%s.png"%name
    cv.SaveImage(os.path.join(image_path,imagename), image)
    
    #save annotation
    f = open(os.path.join(annotation_path,"%s.xml"%name),"w")
    f.write(create_annotation(imagename,image,mask_array))
    f.close()



        
    

def display_masks(image, mask_array, cv_bridge):
    for mask in mask_array.masks:
        img = cv.GetImage(image)
        pt1 = (mask.roi.x,mask.roi.y)
        pt2 = (mask.roi.x+mask.roi.width,mask.roi.y+mask.roi.height)
        cv.Rectangle(image,pt1,pt2,cv.Scalar(255,0,0),2)
        if len(mask.mask.data) != 0:
            cv.SetImageROI(img,(mask.roi.x,mask.roi.y,mask.roi.width,mask.roi.height))
            cv_mask = cv_bridge.imgmsg_to_cv(mask.mask)
            cv.AddS(img,(0,50.0,0),img,cv_mask)
            cv.ResetImageROI(img)



class DisplayFrame(wx.Frame):

    def __init__(self, *args, **kwds):
        wx.Frame.__init__(self, *args, **kwds)
        self.image = OpenCVImage(self, -1, style=wx.TAB_TRAVERSAL)
        self.SetSize((640,480))
        self.SetTitle("Displayer")

        rospy.init_node('display_gui', anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        image_sub = message_filters.Subscriber("image",Image)
        masks_sub = message_filters.Subscriber("masks",MaskArray)
        self.ts = message_filters.TimeSynchronizer([image_sub, masks_sub], 100)
        self.ts.registerCallback(self.callback)


    def callback(self,image,mask_array):
        try:
            cv_image = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError, e:
            print e
        if check_valid(cv_image,mask_array):
            if save_path != '':
                save_image_and_masks(str(image.header.stamp), cv_image, mask_array)
        display_masks(cv_image,mask_array,self.bridge)
        self.image.SetImage(cv_image)
        #pdb.set_trace()
        #cv.SaveImage("images/%s.png"%str(image.header.stamp), cv_image)


    def message(self, str):
        rospy.loginfo(str)


if __name__ == "__main__":

    argv = rospy.myargv()
    if len(argv)==3:
        save_path = argv[1]
        class_name = argv[2]
        rospy.loginfo("Saving image annotations for class: %s to %s"%(class_name, save_path))

    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame = DisplayFrame(None, -1, "")
    app.SetTopWindow(frame)
    frame.Show()
    app.MainLoop()
