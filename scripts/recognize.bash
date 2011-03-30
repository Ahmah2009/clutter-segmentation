#!/usr/bin/env bash

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: recognize.bash <image-file>

Tries to recognize objects on a image file, using the tod kinect training base.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi
tmpf=$(mktemp)
rosrun tod_detecting recognizer --image=$1 \
    --tod_config=$CLUTSEG_PATH/base/config.yaml \
    --base=$CLUTSEG_PATH/base \
    --verbose=1 | grep "Object name" > $tmpf
zenity --text-info --filename=$tmpf --title "Results"
rm $tmpf

