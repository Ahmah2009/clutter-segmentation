#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: ias-kinect-test-train

Evaluates classifier trained on subjects from ias kinect database using
ias_kinect_test_all as testing set.
HELP
    exit
fi

source ~/.profile
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p ias_kinect_test_all/result
    link-test-results -u ias_kinect_test_all
    rm -f ias_kinect_test_all/result/*
    blackbox_recognizer \
        -B ias_kinect_train \
        -I ias_kinect_test_all \
        --store=ias_kinect_test_all/result \
        --testdesc=ias_kinect_test_all/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f configs/config.minInliersCount_25.maxProjectionError_12.yaml
    link-test-results ias_kinect_test_all
popd > /dev/null
