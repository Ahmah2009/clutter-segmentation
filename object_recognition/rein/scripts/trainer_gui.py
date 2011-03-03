#!/usr/bin/env python

PACKAGE='rein'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rein.srv import StartTraining, TrainInstance, TrainInstanceRequest, EndTraining
from rein.msg import MaskArray, Mask
import message_filters
import cv

import threading


WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)


import wx
import trainer_frame

import pdb


def subscribe_service(name,type):
    rospy.loginfo("Waiting for service: %s"%name)
    rospy.wait_for_service(name)
    return rospy.ServiceProxy(name,type)


def display_masks(image, mask_array, cv_bridge):
    for mask in mask_array.masks:
        img = cv.GetImage(image)
        pt1 = (mask.roi.x,mask.roi.y)
        pt2 = (mask.roi.x+mask.roi.width,mask.roi.y+mask.roi.height)
        cv.Rectangle(image,pt1,pt2,cv.Scalar(255,0,0),2)
        if len(mask.mask.data)!=0:
            cv.SetImageROI(img,(mask.roi.x,mask.roi.y,mask.roi.width,mask.roi.height))
            cv_mask = cv_bridge.imgmsg_to_cv(mask.mask)
            cv.AddS(img,(0,50.0,0),img,cv_mask)
            cv.ResetImageROI(img)



class TrainerFrame(trainer_frame.TrainerFrame):

    def __init__(self, *args, **kwds):
        trainer_frame.TrainerFrame.__init__(self, *args, **kwds)
        self.training_lock = threading.Lock()
        self.trainer_statusbar.SetStatusText("Ready",0)

        rospy.init_node('trainer_gui', anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        image_sub = message_filters.Subscriber("image",Image)
        masks_sub = message_filters.Subscriber("masks",MaskArray)
        self.ts = message_filters.TimeSynchronizer([image_sub, masks_sub], 10)
        self.ts.registerCallback(self.callback)

        self.start_training = subscribe_service("start_training",StartTraining)
        self.train_instance = subscribe_service("train_instance",TrainInstance)
        self.end_training = subscribe_service("end_training",EndTraining)
        rospy.loginfo("Subscribed to all services")

        model_name = rospy.get_param("~model","")
        self.name_ctrl.SetValue(model_name)

        self.setTrainingMode(False)


    def callback(self,image,mask_array):
        self.training_lock.acquire()
        try:
            cv_image = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError, e:
            print e
        #pdb.set_trace()
        #cv.SaveImage("images/%s.png"%str(image.header.stamp), cv_image)
        try:
            if self.training:
                req = TrainInstanceRequest()
                req.name = str(self.name_ctrl.GetValue())
                req.image = image
                if len(mask_array.masks)>0:
                    req.mask = mask_array.masks[0]
                else:
                    req.mask = Mask()
                try:
                    print "Calling train_instance"
                    self.train_instance(req)
                except rospy.ServiceException, e:
                    wx.MessageBox(str(e),"Error", wx.OK | wx.ICON_ERROR, self)
        except Exception, e:
            rospy.logdebug(e)
        display_image = cv_image
        if display_image.channels==1:
            display_image = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC3)
            cv.CvtColor(cv_image, display_image, cv.CV_GRAY2RGB)
        display_masks(display_image,mask_array,self.bridge)
        self.image.SetImage(display_image)
        self.training_lock.release()


    def message(self, str):
        self.trainer_statusbar.SetStatusText(str,0)
        rospy.loginfo(str)


    def setTrainingMode(self,state):
        self.training = state
        self.name_ctrl.Enable(not state)
        self.start_training_button.Enable(not state)
        self.save_model_button.Enable(state)
        self.cancel_training_button.Enable(state)


    def startTraining(self,event):
        name = str(self.name_ctrl.GetValue())
        if not name:
            wx.MessageBox("You must first give a name to the trained class","Error", wx.OK | wx.ICON_ERROR, self)
            return
        try:
            self.start_training(name)
            self.message("Training class: %s"%name)
            self.setTrainingMode(True)
        except rospy.ServiceException, e:
            wx.MessageBox(str(e),"Error", wx.OK | wx.ICON_ERROR, self)

        
    def cancelTraining(self,event):
        self.message("Training stopped")
        self.setTrainingMode(False)

    def saveModel(self,event):
        self.training_lock.acquire()
        name = str(self.name_ctrl.GetValue())
        try:
            self.end_training(name)
            self.message("Saved model: %s"%name) 
            self.setTrainingMode(False)
        except rospy.ServiceException, e:
            wx.MessageBox(str(e),"Error", wx.OK | wx.ICON_ERROR, self)
        self.training_lock.release()


if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    trainer = TrainerFrame(None, -1, "")
    app.SetTopWindow(trainer)
    trainer.Show()
    app.MainLoop()
