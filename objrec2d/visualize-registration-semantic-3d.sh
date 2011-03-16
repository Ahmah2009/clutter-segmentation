#!/bin/sh

if [ "$1" = "" ]; then
    echo "Usage: visualize-registration-semantic-3d.sh <object>"
    exit 1
fi

obj=$1

cd build/semantic-3d/$obj.delimited.rotated.pcd
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

