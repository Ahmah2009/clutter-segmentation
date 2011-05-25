#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: result-montage <artifact> <experiment-id>

Constructs a collage picture from experiment results. Requires variable
CLUTSEG_RESULT_DIR to be set in the environment. Generated pictures will be
saved to CLUTSEG_ARTIFACT_DIR (which should also be present in your
environment).
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if [ "$CLUTSEG_RESULT_DIR" = "" ] ; then
    usage
    echo "CLUTSEG_RESULT_DIR not set."
    exit 1
fi

if [ "$CLUTSEG_ARTIFACT_DIR" = "" ] ; then
    usage
    echo "CLUTSEG_ARTIFACT_DIR not set."
    exit 1
fi

expect_arg 0
expect_arg 1

artifact=$(get_arg 0)
exp=$(printf "%05d" $(get_arg 1))
images=$(find $CLUTSEG_RESULT_DIR/$exp -iname "*.locate_choice.png" | sort)
outfile=$CLUTSEG_ARTIFACT_DIR/locate_choice.collage.$exp.jpg
tile=
if [ "$(echo $images | wc --words)" = "21" ] ; then
    tile="-tile 7x3"
fi
montage $tile -geometry 625x500+7+7 $images $outfile
eog $outfile
