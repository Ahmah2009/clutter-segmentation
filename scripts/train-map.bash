#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-detect <base>

Detects features from training images given pose and masks.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg $1

base=$1
pushd $CLUTSEG_PATH/$base > /dev/null
    assert_training_base
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Detecting features $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training detector..."
            rosrun tod_training detector -d $subj -j$JOBS
        fi
    done
popd > /dev/null

