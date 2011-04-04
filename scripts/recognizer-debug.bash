#!/usr/bin/env bash
#

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: recognizer-debug.bash

Runs the recognizer executable of tod_detecting in GNU debugger. Might be
useful for quick regression tests.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pkg_tod_detecting=$(rospack find tod_detecting)
gdb --args $pkg_tod_detecting/bin/recognizer \
    --base=$CLUTSEG_PATH/tod_kinect_train_9 \
    --image=$CLUTSEG_PATH/tod_kinect_train_9/fat_free_milk/image_00000.png \
    --tod_config=$CLUTSEG_PATH/tod_kinect_train_9/config.yaml \
    --verbose=1

