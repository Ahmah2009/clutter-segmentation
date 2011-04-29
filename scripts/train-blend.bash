#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-blend <base> [--keypoints]

Creates blend images from masks and images. If an image has no mask (e.g. pose
has not been estimated) then it will be skipped. If flag --keypoints is set,
this script will create blend images for mask and keypoint images. The
keypoints images are assumed to exist. You can create them first by invoking
train-keypoints.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg 0

base=$(get_arg 0)
pushd $CLUTSEG_PATH/$base > /dev/null
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Generating blend images for $subj"
            echo "--------------------------------------------------------"
            pushd $subj >/dev/null
                for img in image_?????.png ; do
                    if [ -f $img.mask.png ] ; then
                        if has_opt --keypoints ; then
                            if [ -f $img.keypoints.png ] ; then 
                                echo "Creating keypoints blend image for $subj/$img ..."
                                composite $img.keypoints.png $img.mask.png -blend %60 $img.mask.blend.png
                            else
                                echo "Skipping $subj/$img, no keypoints image."
                            fi
                        else
                            if [ -f $img.png ] ; then 
                                echo "Creating blend image for $subj/$img ..."
                                composite $img.png $img.mask.png -blend %60 $img.mask.blend.png
                            else
                                echo "Skipping $subj/$img, only mask exists."
                            fi
                        fi
                    else
                        echo "Skipping $subj/$img, no mask."
                    fi 
                done
            popd > /dev/null
        fi
    done
popd > /dev/null
