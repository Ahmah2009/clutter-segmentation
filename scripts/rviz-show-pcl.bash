#!/usr/bin/env bash

# TODO: decide whether this is helpful for Dejan
# TODO: find one single term to fix problems with name of training data and training base

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: rviz-show-pcl.bash

Plays a training bag for the downy training subject in the tod kinect training
dataset using rosbag and shows everything in rviz. No further configuration is
necessary. Replay must be started by hitting <space> in the terminal.
HELP
    exit
fi

source ~/.profile
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

if [ ! "$(pgrep rosout)" ] ; then
    echo "WARNING: Master server roscore does not seem to be running."
fi 

# Start rviz
rosrun rviz rviz --display-config $CLUTSEG_PATH/clutter-segmentation/misc/pcl_view_training.vcg &
echo "Sleeping for 3 seconds to give rviz a headstart ..."
sleep 3
rosbag play $CLUTSEG_PATH/tod_kinect_bags/downy.bag $CLUTSEG_PATH/tod_kinect_bags/downy.tf.bag --pause

