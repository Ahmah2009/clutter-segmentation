#!/usr/bin/env bash

cp CMakeLists.txt CMakeLists.txt.backup
cp CMakeListsTest.txt CMakeLists.txt

make tests
if [ "$?" = "0" ] ; then
    ./bin/utest --gtest_filter=$1
fi

mv CMakeLists.txt.backup CMakeLists.txt

