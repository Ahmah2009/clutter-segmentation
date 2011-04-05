#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: s3dbase-extract-features.bash <training-subject>

Extracts features using tod_training detector and f3dcreator, thus running the
last two steps of the tod_training pipeline without estimating poses or
creating masks.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH/s3dbase-single > /dev/null
    if [ ! -e $1/camera.yml ];then
        echo "ERROR: Data for training subject not found."
        exit
    fi

    echo "detector"
    rosrun tod_training detector -d $1 --verbose=true
    echo "f3d"
    rosrun tod_training f3d_creator -d $1 --verbose=true
popd > /dev/null

