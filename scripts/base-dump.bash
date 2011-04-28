#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-dump <base>

Dumps image and point clouds from bags. Besides, no other files will be deleted
or replaced in the training base.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg $1

base=$1
pushd $CLUTSEG_PATH/$base > /dev/null
    for bag in *.bag; do
        subj=$(basename $bag .bag)
        echo "Dumping $subj"
        echo "--------------------------------------------------------"
        echo "Running tod_training bag_dumper ..."
        rosrun tod_training bag_dumper -P . -N $subj -B $bag --image=image_color
    done
popd > /dev/null

