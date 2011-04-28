#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-mask <base> [--fix] 

Performs masking. If flag '--fix' is given the masks are preprocessed, an
attempt to remove areas in the mask outside of the actual region of interest.
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
            echo "Masking $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training masker ..."
            rosrun tod_training masker -d $subj -M 1 -j8
            if has_opt "--fix" ; then
                pushd $subj >/dev/null
                    for mask in image_?????.png.mask.png ; do
                        echo "Fixing $subj/$mask ..."
                        convert $mask -morphology Open Disk:10.3 $mask
                        fixmask $mask $mask
                        convert $mask -morphology Dilate Disk:2.3 $mask
                    done
                popd > /dev/null
            fi
       fi
    done
popd > /dev/null
