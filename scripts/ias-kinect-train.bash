#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: ias-kinect-train [--no-dump]

Creates a training base from IAS kinect bags.
USAGE
}

source ~/.env
source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if has_no_arg --no-dump $*; then
    rosrun tod_training dump_all.py $CLUTSEG_PATH/ias_kinect_bags/ $CLUTSEG_PATH/ias_kinect_train/
fi
pushd $CLUTSEG_PATH/ias_kinect_train > /dev/null
    rosrun tod_training train_all.sh
popd > /dev/null

