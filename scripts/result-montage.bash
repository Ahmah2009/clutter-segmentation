#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: result-montage <artifact> <experiment-id> [--show]

Constructs a collage picture from experiment results. Requires variable
CLUTSEG_RESULT_DIR to be set in the environment. Generated pictures will be
saved to CLUTSEG_ARTIFACT_DIR (which should also be present in your
environment).
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*
source $CLUTSEG_PATH/experiment_setup.bash

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

function tile() {
    if [ "$(echo $* | wc --words)" = "21" ] ; then
        echo "-tile 7x3"
    fi
}

# first argument currently ignored
artifact=$(get_arg 0)
exp=$(printf "%05d" $(get_arg 1))
geometry="-geometry 625x500+7+7"
out=$CLUTSEG_ARTIFACT_DIR/$artifact.$exp.jpg
if [ "$artifact" = "locate_choice.collage" ] ; then
    lcis=$(find $CLUTSEG_RESULT_DIR/$exp -iname "*.locate_choice.png" | sort)
    montage $(tile $lcis) $geometry $lcis $out
    if has_opt --show ; then
        eog $out
    fi
elif [ "$artifact" = "detect_choices.collage" ] ; then
    dcis=$(find $CLUTSEG_RESULT_DIR/$exp -iname "*.detect_choices.png" | sort)
    montage $(tile $dcis) $geometry $dcis $out
    if has_opt --show ; then
        eog $out
    fi
else
    echo "Valid artifacts are 'locate_choice.collage' and 'detect_choices.collage'"
    exit 1
fi
