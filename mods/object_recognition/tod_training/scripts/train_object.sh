#!/bin/bash
if [ ! -e $1/camera.yml ];then
	echo "usage: train_object.sh object_path"
	echo "be sure to run this from the root path of your training base"
	exit
fi
if [ ! $JOBS ]; then
JOBS=8
fi
echo "pose"
rosrun tod_training pose_estimator -d $1 -j1
echo "masker"
rosrun tod_training masker -d $1 -M 1 -j$JOBS
for mask in $1/*.mask.png ; do
    echo "Fixing mask $mask"
    convert $mask -morphology Open Disk:10.3 $mask
    fixmask $mask $mask
done
#0 is for 3d box, #1 is for pcl segmentation
echo "detector"
rosrun tod_training detector -d $1 -j$JOBS
echo "f3d"
rosrun tod_training f3d_creator -d $1 -j$JOBS
