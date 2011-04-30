#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: train-detect <base> [--verbose] [--debug] [--serial}

Detects features from training images given pose and masks.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg 0
base=$(get_arg 0)

if has_opt --verbose ; then
    verbose="--verbose=1"
fi

if has_opt --debug ; then
    debug="gdb --args "
fi

jobs=8
if has_opt --serial ; then
    jobs=1
fi

pushd $CLUTSEG_PATH/$base > /dev/null
    assert_base
    for d in *; do
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Extract features for $subj"
            echo "--------------------------------------------------------"
            echo "Running tod_training detector ..."
            $debug $(rospack find tod_training)/bin/detector -d $subj -j$jobs $verbose
        fi
    done
popd > /dev/null

