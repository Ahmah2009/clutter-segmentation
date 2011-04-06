#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-recog-dbg

Recognizes objects on a test image.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pkg_tod_detecting=$(rospack find tod_detecting)
pushd $CLUTSEG_PATH > /dev/null
    gdb --args $pkg_tod_detecting/bin/recognizer --base=ias_kinect_train --image=$1 --tod_config=ias_kinect_train/config.yaml --verbose=2
popd > /dev/null
