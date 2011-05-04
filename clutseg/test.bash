#!/usr/bin/env bash

cp CMakeLists.txt CMakeLists.txt.backup
cp CMakeListsTest.txt CMakeLists.txt

if [ "$1" = "tests" ]; then
    make tests
elif [ "$1" = "test" ]; then
    shift 1
    ./bin/utest $*
else
    cat <<HELP
Usage: test.bash TARGET
Available targets:
    tests -- compile and link tests
    test  -- run tests
HELP
fi

mv CMakeLists.txt.backup CMakeLists.txt

