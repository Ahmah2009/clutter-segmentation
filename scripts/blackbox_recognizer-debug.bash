#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: blackbox_recognizer-debug.bash

Runs the blackbox recognizer in GNU debugger, using tod kinect dataset.  Might
be useful for quick regression tests or also for demonstration purposes.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
pkg_clutseg_util=$(rospack find clutseg_util)
gdb --args $pkg_clutseg_util/bin/frecognizer \
    --base=tod_kinect_train/ \
    --tod_config=tod_kinect_train/config.yaml \
    --image=test \
    --log=blackbox_recognizer-debug.log \
    --result=blackbox_recognizer-debug.result.txt \
    --verbose=1
popd > /dev/null

