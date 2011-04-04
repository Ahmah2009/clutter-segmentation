#!/usr/bin/env python
"""Some unit tests for poserandomization.py"""

from poserandomization import *

def test_load_pose():
    pose = PoseRT()
    # FIXME: remove absolute path
    p = os.getenv("CLUTSEG_PATH")
    pose.read(os.path.join(p, "clutter-segmentation/clutseg_test/data/pose.yaml"))
    assert pose.rvec == [ 2.00, 4.00, 8.00 ]
    assert pose.tvec == [ 16.00, 32.00, 64.00 ]

def test_listdir():
    p = os.getenv("CLUTSEG_PATH")
    files = os.listdir(os.path.join(p, "tod_kinect_train_9/downy/"))
    assert "image_00003.png.pose.yaml" in files
    
def test_replace_data():
    exp_old = "pose: rvec: !!opencv-matrix rows: 3 cols: 1 data: [ 5.00, 2.90, 6.70 ]"
    rvec = [2.00, 4.00, 8.00]
    exp_new = "pose: rvec: !!opencv-matrix rows: 3 cols: 1 data: [2.0, 4.0, 8.0]"
    assert exp_new == replace_data(exp_old, "rvec", rvec)

def test_get_env_var():
    assert os.getenv("CLUTSEG_PATH") != ""

def test_gzip_decompress():
    p = os.getenv("CLUTSEG_PATH")
    f = gzip.open(os.path.join(p, "tod_kinect_train_9", "downy", "image_00000.png.features.yaml.gz"), "rb")
    f.close()

def test_gzip_compressed_name():
    p = os.getenv("CLUTSEG_PATH")
    (h, t) = tempfile.mkstemp()
    f = gzip.open(t, "wb")
    f.write("foo bar")
    f.close()


def test_gzip_compress():
    p = os.getenv("CLUTSEG_PATH")
    (h, t) = tempfile.mkstemp()
    f = gzip.open(t, "wb")
    f.write("foo bar")
    f.close()

