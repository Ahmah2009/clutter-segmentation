#!/bin/sh
#
#

bags=build/bags
base=build/base

if [ "$1" = "--help" ] || [ "$1" = "-h" ] ;
then
    echo "Usage: gen_training_base.sh <path-to-bags> <path-to-base>"
    echo ""
    echo "Generates a training base from a set of existing bags."
    echo "<path-to-bags> contains bag files as can be downloaded"
    echo "from http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training"
    echo "plus the required configuration files."
fi

# rosrun tod_training dump_all.py $bags $base
cp $bags/fiducial.yml $base
cp $bags/features.config.yaml $base
cp $bags/config.* $base
cd $base
rosrun tod_training train_all.sh --verbose
