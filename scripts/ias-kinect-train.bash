#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-train

Creates a training base from IAS kinect bags.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi


rosrun tod_training dump_all.py $CLUTSEG_PATH/ias_kinect_bags/ $CLUTSEG_PATH/ias_kinect_train/
pushd $CLUTSEG_PATH/ias_kinect_train > /dev/null
    rosrun tod_training train_all.sh
popd > /dev/null

