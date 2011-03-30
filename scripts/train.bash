#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: train.bash [--no-dump]

Dumps the downloaded tod kinect training bag files and constructs
a training base using tod_training. This process takes some time.
--no-dump suppresses the bag dumping stage."
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

if [ "$1" != "--no-dump" ] ; then
    # Dump contents of bag files
    rosrun tod_training dump_all.py $CLUTSEG_PATH/bags $CLUTSEG_PATH/base
fi
# Build training base
cp $CLUTSEG_PATH/bags/fiducial.yml b$CLUTSEG_PATH/ase/
cp $CLUTSEG_PATH/bags/features.config.yaml b$CLUTSEG_PATH/ase/
cp $CLUTSEG_PATH/bags/config.yaml b$CLUTSEG_PATH/ase/
cp $CLUTSEG_PATH/bags/config.txt $CLUTSEG_PATH/base/
cd $CLUTSEG_PATH/base
rosrun tod_training train_all.sh

echo "Finished. See $CLUTSEG_PATH/base"

