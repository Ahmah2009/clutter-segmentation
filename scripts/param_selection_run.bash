#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: param_selection_run [--debug|--memcheck]
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

source $CLUTSEG_PATH/experiment_setup.bash

if has_opt --debug ; then
    debug="gdb --args"
elif has_opt --memcheck ; then
    debug="valgrind --tool=memcheck"
fi


pushd $CLUTSEG_PATH/clutter-segmentation/clutseg > /dev/null
    if [ "$?" = 0 ] ; then
        # TODO: create YAML file that couples all information
        # necessary to run an experiment
        nice -n 3 $memcheck $debug param_selection $CLUTSEG_EXPERIMENT_DB $CLUTSEG_TRAIN_CACHE_DIR $CLUTSEG_RESULT_DIR
    fi
popd > /dev/null

