#!/bin/sh
#
# Actually it was a little bit tricky to prevent rviz from
# freezing the whole system for decades, so this is a
# configuration that worked well together with
# pcl_view_training.vcg
rosbag play --pause --rate=0.05 --loop $1.bag $1.tf.bag
