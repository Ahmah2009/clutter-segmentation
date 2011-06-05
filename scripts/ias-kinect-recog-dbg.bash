#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-recog-dbg

Debugs recognizer using data from ias kinect dataset. See ticket 5083 in
wg-ros-pkg trac if getting OpenCV errors.
HELP
    exit
fi

source ~/.profile
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pkg_tod_detecting=$(rospack find tod_detecting)
pushd $CLUTSEG_PATH > /dev/null
    gdb --args $pkg_tod_detecting/bin/recognizer --base=ias_kinect_train --image=ias_kinect_train/haltbare_milch/image_00000.png --tod_config=ias_kinect_train/config.yaml --verbose=0 --mode=1
popd > /dev/null
