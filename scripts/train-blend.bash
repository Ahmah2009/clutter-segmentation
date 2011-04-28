#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-blend <base>

Creates blend images from masks and images. If an image has no mask (e.g. pose
has not been estimated) then it will be skipped.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg $1

base=$1
pushd $CLUTSEG_PATH/$base > /dev/null
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Generating blend images for $subj"
            echo "--------------------------------------------------------"
            pushd $subj >/dev/null
                for img in image_*.png ; do
                    if [ -f $img.mask.png ] ; then
                        echo "Creating blend image for $subj/$img ..."
                        composite $img $img.mask.png -blend %60 $img.mask.blend.png
                    else
                        echo "Skipping $subj/$img, no mask."
                    fi 
                done
            popd > /dev/null
        fi
    done
popd > /dev/null
