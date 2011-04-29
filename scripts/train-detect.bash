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
    assert_base
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Extract features for $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training detector ..."
            rosrun tod_training detector -d $subj -j8
        fi
    done
popd > /dev/null

