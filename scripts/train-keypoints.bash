#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-keypoints <base>

Creates images showing keypoints on training images from feature.yaml.gz files.
Prior to invoking this script, these feature files have to be generated, e.g.
by calling train-detect.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg $1

base=$1
pushd $CLUTSEG_PATH/$base > /dev/null
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Generating keypoint images for $subj"
            echo "--------------------------------------------------------"
            pushd $subj >/dev/null
                for img in image_?????.png ; do
                    if [ -f $img.features.yaml.gz ] ; then
                        echo "Creating keypoints image for $subj/$img ..."
                        keypoints_image $img $img.features.yaml.gz $img.keypoints.png
                    else
                        echo "Skipping $subj/$img, no feature.yaml.gz found."
                    fi 
                done
            popd > /dev/null
        fi
    done
popd > /dev/null
