#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-get

Downloads IAS kinect training bags from http://ias.cs.tum.edu/~pangerci
HELP
    exit
fi

source ~/.profile
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH/ias_kinect_bags > /dev/null
    wget --input-file $CLUTSEG_PATH/clutter-segmentation/misc/ias-kinect-bags.wget --base http://ias.cs.tum.edu/~pangerci/
popd > /dev/null

