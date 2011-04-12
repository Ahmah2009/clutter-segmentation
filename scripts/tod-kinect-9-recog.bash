#!/usr/bin/env bash

# 2011-04-04: working 

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: recognize.bash <image-file>

Tries to recognize objects on a image file, using a classifier trained on the
tod kinect training 9 dataset.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi
tmpf=$(mktemp)
rosrun tod_detecting recognizer --image=$1 \
    --tod_config=$CLUTSEG_PATH/tod_kinect_train_9/config.yaml \
    --base=$CLUTSEG_PATH/tod_kinect_train_9 \
    --verbose=1 --mode=1 | grep "Object name" > $tmpf
zenity --text-info --filename=$tmpf --title "Results"
rm $tmpf

