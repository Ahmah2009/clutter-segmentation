#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: frecognizer-debug.bash

Runs the folder recognizer executable of tod_detecting in GNU debugger. Might
be useful for quick regression tests.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pkg_tod_detecting=$(rospack find tod_detecting)
gdb --args $pkg_tod_detecting/bin/frecognizer \
    --base=$CLUTSEG_PATH/base/ \
    --tod_config=$CLUTSEG_PATH/base/config.yaml \
    --image=$CLUTSEG_PATH/test \
    --log=$CLUTSEG_PATH/tmp/test.deleteme.log \
    --verbose=1

