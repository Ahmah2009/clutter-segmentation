#!/bin/bash

# Example directory containing _vfh.pcd files
DATA=`pwd`/data

# Inlier distance threshold
thresh=100

# Chi-Square distance metric
metric=7

# Get the closest K nearest neighbors
k=12

for i in `find $DATA -type d -name "*"`
do
  echo $i
  for j in `find $i -type f \( -iname "*cluster*_vfh.pcd" \) | sort -R`
  do
    rosrun vfh_cluster_classifier nearest_neighbors -k $k -thresh $thresh -metric $metric $j -cam "0.403137,0.868471/0,0,0/-0.0932051,-0.201608,-0.518939/-0.00471487,-0.931831,0.362863/1464,764/6,72"
    break
  done
done
