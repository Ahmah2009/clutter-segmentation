#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-detect <base>

Maps 2d keypoints to their corresponding 3d points in training image.
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
            echo "Mapping 2d-3d: $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training f3d_creator ..."
            rosrun tod_training f3d_creator -d $subj -j8
        fi
    done
popd > /dev/null

