#!/usr/bin/env bash

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: rosbag-play.bash <training-subject>

Replays a training bag for a certain training subject in the tod kinect
training dataset using rosbag. Replay must be started by hitting <space> in the
terminal.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

if [ ! "$(pgrep rosout)" ] ; then
    echo "WARNING: Master server roscore does not seem to be running."
fi 

rosbag play --pause --rate=0.05 --loop $CLUTSEG_PATH/tod_kinect_bags/$1.bag $CLUTSEG_PATH/tod_kinect_bags/$1.tf.bag

