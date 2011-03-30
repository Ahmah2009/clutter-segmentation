#!/usr/bin/env bash

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: semantic3d-visualize.bash <training-subject> <angle> 

Displays the point clouds associated with a training subject from a certain
angle.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

obj=$1
angle=$2

cd $C 
# workaround for bug https://code.ros.org/trac/wg-ros-pkg/ticket/5072
a=$(mktemp --suffix .pcd)
b=$(mktemp --suffix .pcd)
c=$(mktemp --suffix .pcd)

cp $CLUTSEG_PATH/semantic3d-data/$obj.delimited.rotated.pcd/${obj}_${angle}_.log.delimited.rotated.pcd $a
cp $CLUTSEG_PATH/semantic3d-data/$obj.delimited.pcd/${obj}_${angle}_.log.delimited.pcd $b
cp $CLUTSEG_PATH/semantic3d-data/$obj.pcd/${obj}_${angle}_.log.pcd $c

rosrun pcl_visualization pcd_viewer -fc 255,0,0 $a -fc 0,255,0 $b -f 0,0,255 $c -ax 0.1 

rm $a
rm $b
rm $c
