#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: blackbox_recognizer-demo.bash [-g]

Runs the blackbox recognizer, using tod kinect dataset.  Might be useful for
quick regression tests or also for demonstration purposes. Set -g flag for
starting in GNU debugger.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

flag_debug=

while getopts 'g' OPTION
do
    case $OPTION in
    g)  flag_debug=1
        ;;
    esac
done

# TODO: rename test-truth.txt to testdesc.tx
pushd $CLUTSEG_PATH > /dev/null
    if [ "$flag_debug" ]
    then
        cmd="gdb --args"
    fi
    pkg_clutseg_util=$(rospack find clutseg_util)
    $cmd $pkg_clutseg_util/bin/blackbox_recognizer \
        --base=tod_kinect_train/ \
        --tod_config=tod_kinect_train/config.yaml \
        --image=tod_kinect_test \
        --testdesc=tod_kinect_test/test-truth.txt \
        --log=blackbox_recognizer-demo.log \
        --result=blackbox_recognizer-demo.result.txt \
        --roc=blackbox_recognizer-demo.roc.gnuplot\
        --verbose=1
popd > /dev/null

