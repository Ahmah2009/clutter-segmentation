#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: param_selection_test [--debug] 

See script for documentation.
USAGE
}

source ~/.env
source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if has_opt --debug ; then
    debug="gdb --args"
fi

pushd $CLUTSEG_PATH/clutter-segmentation/clutseg > /dev/null
    if [ "$?" = 0 ] ; then
        cp data/paramsel.sqlite3 build
        mkdir -p build/train_cache -p build/results
        $debug bin/experiment_runner build/paramsel.sqlite3 build/train_cache build/results
    fi
popd > /dev/null

