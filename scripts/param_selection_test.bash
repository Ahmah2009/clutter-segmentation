#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: param_selection_test [--debug] 

See script for documentation.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if has_opt --debug ; then
    debug="gdb --args"
fi

pushd $CLUTSEG_PATH/clutter-segmentation/clutseg > /dev/null
    make
    cp data/paramsel.sqlite3 build
    $debug param_selection build/paramsel.sqlite3 build/train_cache
popd > /dev/null

