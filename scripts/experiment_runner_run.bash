#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: param_selection_run [--debug|--memcheck]
USAGE
}

source ~/.env
source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*
source $CLUTSEG_PATH/experiment_setup.bash

if has_opt --debug ; then
    debug="gdb --args"
elif has_opt --memcheck ; then
    debug="valgrind --tool=memcheck"
fi


pushd $CLUTSEG_PATH/clutter-segmentation/clutseg > /dev/null
    if [ "$?" = 0 ] ; then
        nice -n 1 $memcheck $debug bin/experiment_runner $CLUTSEG_EXPERIMENT_DB $CLUTSEG_TRAIN_CACHE_DIR $CLUTSEG_RESULT_DIR
    fi
popd > /dev/null

