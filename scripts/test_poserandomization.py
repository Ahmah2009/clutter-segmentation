#!/usr/bin/env python

from poserandomization import *

def test_load_pose():
    pose = PoseRT()
    pose.read("/home/julius/Studium/BA/clutter-segmentation/clutseg_test/data/pose.yaml")
    assert pose.rvec == [ 2.00, 4.00, 8.00 ]
    assert pose.tvec == [ 16.00, 32.00, 64.00 ]

def test_listdir():
    files = os.listdir("/home/julius/Studium/BA/tod_kinect_train/downy/")
    assert "image_00003.png.pose.yaml" in files
    
def test_replace_data():
    exp_old = "pose: rvec: !!opencv-matrix rows: 3 cols: 1 data: [ 5.00, 2.90, 6.70 ]"
    rvec = [2.00, 4.00, 8.00]
    exp_new = "pose: rvec: !!opencv-matrix rows: 3 cols: 1 data: [2.0, 4.0, 8.0]"
    assert exp_new == replace_data(exp_old, "rvec", rvec)

