#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: train.bash [--no-dump]

Dumps the downloaded tod kinect training bag files and constructs
a training base using tod_training. This process takes some time.
--no-dump suppresses the bag dumping stage."
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
    if [ "$1" != "--no-dump" ] ; then
        # Dump contents of bag files
        rosrun tod_training dump_all.py tod_kinect_bags tod_kinect_train
    fi
    # Build training tod_kinect_train
    cp tod_kinect_bags/fiducial.yml tod_kinect_train/
    cp tod_kinect_bags/features.config.yaml tod_kinect_train/
    cp tod_kinect_bags/config.yaml tod_kinect_train/
    cd tod_kinect_train
    rosrun tod_training train_all.sh

    echo "Finished. See $CLUTSEG_PATH/tod_kinect_train"
popd > /dev/null
