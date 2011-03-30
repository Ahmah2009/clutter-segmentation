#!/usr/bin/env bash
#
# TODO: load valid configuration into rviz
# TODO: decide whether this is helpful for Dejan
# TODO: test
# TODO: fix broken paths
# TODO: make use of exported environment variable to fix path issues
# TODO: find one single term to fix problems with name of training data and training base
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

if [ ! "$(pgrep rosout)" ] ; then
    echo "WARNING: Master server roscore does not seem to be running."
fi 

# Start rviz
rosrun rviz rviz --display-config $CLUTSEG_PATH/clutter-segmentation/objrec2d/pcl_view_training.vcg &
echo "Sleeping for 3 seconds to give rviz a headstart ..."
sleep 3
rosbag play $CLUTSEG_PATH/bags/downy.bag $CLUTSEG_PATH/bags/downy.tf.bag --pause

