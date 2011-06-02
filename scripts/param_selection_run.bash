#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: param_selection_run [--debug] 
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if has_opt --debug ; then
    debug="gdb --args"
fi

pushd $CLUTSEG_PATH/clutter-segmentation/clutseg > /dev/null
    mods-link
    rosmake clutseg
    if [ "$?" = 0 ] ; then
        # TODO: create YAML file that couples all information
        # necessary to run an experiment
        nice -n 15 $debug param_selection $CLUTSEG_EXPERIMENT_DB $CLUTSEG_TRAIN_CACHE_DIR $CLUTSEG_RESULT_DIR $CLUTSEG_POST_RUN_CMD
    fi
popd > /dev/null

