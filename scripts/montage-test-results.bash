#!/usr/bin/env bash

function usage() {
    cat <<HELP
Usage: montage-test-results <test-base>

Creates montages using imagemagick for a test result
HELP
}

source "$CLUTSEG_PATH/clutter-segmentation/scripts/base.bash" $*

testbase=$CLUTSEG_PATH/$1

if [ "$testbase" = "" ] ; then
    usage
    exit
fi

function do_montage() {
    pushd $testbase/result > /dev/null
        for a in $* ; do
            pushd $a > /dev/null
               echo "Imagemagick montage $a ..."
               rm -f $a.png
               montage *.png -geometry +10+10 $a.png
            popd > /dev/null
        done
    popd > /dev/null
}

do_montage guesses-all guesses-single matches-merged projection keypoints

