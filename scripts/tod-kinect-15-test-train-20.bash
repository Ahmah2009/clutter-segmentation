#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-test-train

Evaluates classifier trained on subjects from tod kinect database using
tod_kinect_test_train_20 (training images!) as testing set.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p tod_kinect_test_train_20/result
    rm -f tod_kinect_test_train_20/result/*
    blackbox_recognizer \
        -B tod_kinect_train_15 \
        -I tod_kinect_test_train_20 \
        --store=tod_kinect_test_train_20/result \
        --testdesc=tod_kinect_test_train_20/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f configs/config.minInliersCount_25.yaml
popd > /dev/null
