#!/usr/bin/env bash

cp CMakeLists.txt CMakeLists.txt.backup
cp CMakeListsTest.txt CMakeLists.txt

make tests
./bin/utest --gtest_filter=$1

mv CMakeLists.txt.backup CMakeLists.txt

