#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

# Chi-Square distance metric
metric=7

# Build a KD-Tree
files=`find $DATA -type f \( -iname "*cluster*_vfh.pcd" \)`
rosrun vfh_cluster_classifier build_tree $files -metric $metric -linear 0

