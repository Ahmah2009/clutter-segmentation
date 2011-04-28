#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: test-eval <train-base> <test-config> <test-base>

Runs the given classifier on a testing set.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg 0
expect_arg 1
expect_arg 2
trainbase=$(get_arg 0)
config=$(get_arg 1)
testbase=$(get_arg 2)

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p $testbase/result
    rm -f $testbase/result/*
    echo "trainbase=$trainbase"
    echo "config=$config"
    echo "testbase=$testbase"
    blackbox_recognizer \
        -B $trainbase \
        -I $testbase \
        --store=$testbase/result \
        --testdesc=$testbase/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f $config
    link-test-results $testbase 
popd > /dev/null
