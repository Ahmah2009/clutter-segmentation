#!/usr/bin/env bash

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: semantic3d-visualize-registration.bash <training-subject>

Shows all rotated, translated and delimitated point clouds of one training
subject.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

obj=$1

pushd $CLUTSEG_PATH/semantic3d-data/$obj.delimited.rotated.pcd > /dev/null
    rosrun pcl_visualization pcd_viewer \
        ${obj}_-180_.log.delimited.rotated.pcd \
        ${obj}_-150_.log.delimited.rotated.pcd \
        ${obj}_-120_.log.delimited.rotated.pcd \
        ${obj}_-90_.log.delimited.rotated.pcd \
        ${obj}_-60_.log.delimited.rotated.pcd \
        ${obj}_-30_.log.delimited.rotated.pcd \
        ${obj}_0_.log.delimited.rotated.pcd \
        ${obj}_30_.log.delimited.rotated.pcd \
        ${obj}_60_.log.delimited.rotated.pcd \
        ${obj}_90_.log.delimited.rotated.pcd \
        ${obj}_120_.log.delimited.rotated.pcd \
        ${obj}_150_.log.delimited.rotated.pcd \
        ${obj}_180_.log.delimited.rotated.pcd \
        -ax 0.2
popd > /dev/null
