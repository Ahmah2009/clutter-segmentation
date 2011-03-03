#!/bin/bash

# Example directory containing _vfh.pcd files
DATA=`pwd`/data

for i in `find $DATA -type d -name "*"`
do
  echo $i
  for j in `find $i -type f \( -iname "*cluster*_vfh.pcd" \) | sort -n`
  do
    all_files=`echo $all_files" "$j`
    break
  done
done

rosrun pcl_visualization pcd_viewer $all_files
