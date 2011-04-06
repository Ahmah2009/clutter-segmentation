#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: download-tod-kinect-bags.bash

Downloads tod kinect training bags from
http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH/tod_kinect_bags > /dev/null
    wget --input-file $CLUTSEG_PATH/clutter-segmentation/misc/tod-kinect-bags.wget --base  http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/
popd > /dev/null

