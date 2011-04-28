#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-detect <base>

Detects features from training images given pose and masks.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg 0
base=$(get_arg 0)

pushd $CLUTSEG_PATH/$base > /dev/null
    assert_training_base
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Generate 2d-3d mapping for $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training f3d_creator ..."
            rosrun tod_training f3d_creator -d $subj -j$JOBS
        fi
    done
popd > /dev/null

