#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: custom_test_stats.bash

Creates recognition statistics for test images in tod kinect test images
folder, using tod kinect training database.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi


mode=0

res=$CLUTSEG_PATH/test/results.txt
tod_training_pkg=`rospack find tod_training`
tod_detecting_pkg=`rospack find tod_detecting`
echo "Experiment running on "`date` >> $res
echo "--------------------------------------------------" >> $res
echo "--------------------------------------------------" >> $res
svn info $tod_training_pkg >> $res 
echo "--------------------------------------------------" >> $res
svn info $tod_detecting_pkg >> $res 
echo "--------------------------------------------------" >> $res
echo "detection mode: $mode" >> $res
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "config.yaml" >> $res
echo "--------------------------------------------------" >> $res
cat $CLUTSEG_PATH/tod_kinect_train/config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res
echo "features.config.yaml" >> $res
echo "--------------------------------------------------" >> $res
cat $CLUTSEG_PATH/tod_kinect_train/features.config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res
# TODO: remove test-truth.txt
pkg_semantic3d_training=`rospack find semantic3d_training`
python $pkg_semantic3d_training/scripts/recognizer_stats.py -t $CLUTSEG_PATH/test-truth.txt -B $CLUTSEG_PATH/tod_kinect_train/ -f $CLUTSEG_PATH/tod_kinect_train/config.yaml --verbose -m $mode >> $res
echo "" >> $res
