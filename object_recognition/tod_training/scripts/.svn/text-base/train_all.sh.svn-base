#!/bin/bash
if [ ! -e fiducial.yml ]; then
	echo "be sure to run this from the root path of your training base"
	echo "missing fiducial.yml"
	exit
fi
if [ ! -e features.config.yaml ]; then
	echo "be sure to run this from the root path of your training base"
	echo "missing features.config.yml"
	exit
fi 

for file in *; do
   if [ -d $file ]; then
      echo "*****"
      echo $(basename $file)
      rosrun tod_training train_object.sh $(basename $file)
   fi
done