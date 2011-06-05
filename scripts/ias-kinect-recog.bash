#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-recog <image-file>

Recognizes objects on a test image.
HELP
    exit
fi

source ~/.env
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
    clutter-segmentation/scripts/simplerecognizer.py --base ias_kinect_train --image $1 --tod_config ias_kinect_train/config.yaml --mode 1 
popd > /dev/null
