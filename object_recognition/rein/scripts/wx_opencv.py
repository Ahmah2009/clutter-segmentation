#!/usr/bin/env python

import wx
import cv

class OpenCVImage(wx.ScrolledWindow):
    """
    This class implements a wx mage control that can 
    display OpenCV images. It requires the existance
    of the wx.BitmapFromBuffer function (wx ver. 2.8).
    """
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.TAB_TRAVERSAL
        wx.ScrolledWindow.__init__(self, *args, **kwds)

        self.SetSize((640,480))

        self.Bind(wx.EVT_PAINT, self.onPaint, self)
        self.Bind(wx.EVT_SIZE, self.onSize, self)

        self.bmp = None
        self.img = None

    def onPaint(self,event):
        dc = wx.AutoBufferedPaintDC(self)
        dc.Clear()
        if self.bmp!=None:
            dc.DrawBitmap(self.bmp,0,0,False)

    def onSize(self,event):
        if self.img!=None:
            width, height = self.GetClientSizeTuple()
            img2 = cv.CreateMat(height,width,self.img.type)
            cv.Resize(self.img,img2,cv.CV_INTER_NN)
            self.bmp = wx.BitmapFromBuffer(img2.width, img2.height, img2.tostring())
            self.Refresh()


    def SetImage(self, img):
        """
        Sets the image to be displayed on the control. It may be
        called from a different thread than the main GUI thread.
        """
        cv.CvtColor(img, img, cv.CV_BGR2RGB)
        self.img = img
        width, height = self.GetClientSizeTuple()
        img2 = cv.CreateMat(height,width,img.type)
        cv.Resize(self.img,img2,cv.CV_INTER_NN)
        wx.CallAfter(self.updateImage, img2)

    def updateImage(self,image):
        self.bmp = wx.BitmapFromBuffer(image.width, image.height, image.tostring())
        self.Refresh()

