#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-test-train

Evaluates classifier trained on subjects from ias kinect database using
ias_kinect_test_all as testing set.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p ias_kinect_test_all/result
    rm -f ias_kinect_test_all/result/*
    gdb --args blackbox_recognizer \
        -B ias_kinect_train \
        -I ias_kinect_test_all \
        --store=ias_kinect_test_all/result \
        --testdesc=ias_kinect_test_all/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f ias_kinect_train/config.yaml
popd > /dev/null
