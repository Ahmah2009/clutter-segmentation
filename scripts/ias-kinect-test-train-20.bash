#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-test-train

Evaluates classifier trained on subjects from ias kinect database using
ias_kinect_test_train_20 (training images!) as testing set.
HELP
    exit
fi

source ~/.profile
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

# TODO: extract method

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p ias_kinect_test_train_20/result
    link-test-results -u ias_kinect_test_train_20
    rm -f ias_kinect_test_train_20/result/*
    blackbox_recognizer \
        -B ias_kinect_train \
        -I ias_kinect_test_train_20 \
        --store=ias_kinect_test_train_20/result \
        --testdesc=ias_kinect_test_train_20/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f configs/config.minInliersCount_25.yaml
    link-test-results ias_kinect_test_train_20
popd > /dev/null
