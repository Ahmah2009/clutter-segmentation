#!/bin/sh
obj=$1
angle=$2

cd build/semantic-3d/
# workaround for bug https://code.ros.org/trac/wg-ros-pkg/ticket/5072
mkdir -p tmp
cp $obj.delimited.rotated.pcd/${obj}_${angle}_.log.delimited.rotated.pcd tmp/
cp $obj.delimited.pcd/${obj}_${angle}_.log.delimited.pcd tmp/
cp $obj.pcd/${obj}_${angle}_.log.pcd tmp/

rosrun pcl_visualization pcd_viewer -fc 255,0,0 tmp/${obj}_${angle}_.log.delimited.rotated.pcd -fc 0,255,0 tmp/${obj}_${angle}_.log.delimited.pcd -f 0,0,255 tmp/${obj}_${angle}_.log.pcd -ax 0.1 
rm -rf tmp
