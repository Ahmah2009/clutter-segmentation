#!/usr/bin/env bash

function usage() {
    echo "test.bash [--fast] [google options] [gtest_filter]"
}

cp CMakeLists.txt CMakeLists.txt.backup
cp CMakeListsTest.txt CMakeLists.txt

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash
make tests
if [ "$?" = "0" ] ; then
    if has_opt "--fast" ; then
        touch build/fast.flag
    fi
    
    f=$(get_arg 0)
    if [ "$f" = "" ] ; then
       ./bin/utest
    else
       ./bin/utest --gtest_filter=$f
    fi

    if has_opt "--fast" ; then
        rm build/fast.flag
    fi
fi

mv CMakeLists.txt.backup CMakeLists.txt

