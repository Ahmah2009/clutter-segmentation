#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: frecognizer-debug.bash

Runs the folder recognizer executable of tod_detecting in GNU debugger,
using tod kinect training 9 dataset. Might be useful for quick regression tests.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pkg_tod_detecting=$(rospack find tod_detecting)
gdb --args $pkg_tod_detecting/bin/frecognizer \
    --base=$CLUTSEG_PATH/tod_kinect_train_9/ \
    --tod_config=$CLUTSEG_PATH/tod_kinect_train_9/config.yaml \
    --image=$CLUTSEG_PATH/tod_kinect_test_27 \
    --log=$CLUTSEG_PATH/test.deleteme.log \
    --mode=2 \
    --verbose=1

