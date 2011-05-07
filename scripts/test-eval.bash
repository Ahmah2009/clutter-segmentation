#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: test-eval <train-base> <test-config> <test-base> [--debug]

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

if has_opt --debug ; then
    debug="gdb --args"
fi

pushd $CLUTSEG_PATH > /dev/null
    mkdir -p $testbase/result
    rm -rf $testbase/result/*
    $debug blackbox_recognizer \
        -B $trainbase \
        -I $testbase \
        --store=$testbase/result \
        --testdesc=$testbase/testdesc.txt \
        --mode=1 \
        --verbose=0 \
        -f $config
    test-symlink $testbase/result
popd > /dev/null
