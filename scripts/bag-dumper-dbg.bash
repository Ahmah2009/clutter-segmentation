#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: bag-dumper-debug.bash
HELP
    exit
fi

source ~/.env
source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

pkg_tod_training=$(rospack find tod_training)
gdb --args $pkg_tod_training/bin/bag_dumper \
    -P $CLUTSEG_PATH/ias_kinect_train \
    -N jacobs_coffee \
    -B $CLUTSEG_PATH/ias_kinect_bags/jacobs_coffee.bag \
    --image=image_color
