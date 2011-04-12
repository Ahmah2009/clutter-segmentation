#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: test.bash

Creates recognition statistics for test images in tod kinect test 27 images
folder, using tod kinect training 9 database.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi


mode=1

res=$CLUTSEG_PATH/tod_kinect_test_9/results.txt
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
cat $CLUTSEG_PATH/tod_kinect_train_9/config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res
echo "features.config.yaml" >> $res
echo "--------------------------------------------------" >> $res
cat $CLUTSEG_PATH/tod_kinect_train_9/features.config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res
# TODO: remove test-truth.txt
python $CLUTSEG_PATH/clutter-segmentation/scripts/recognizer_stats.py --test $CLUTSEG_PATH/tod_kinect_test_9 -t $CLUTSEG_PATH/tod_kinect_test_9/testdesc.txt -B $CLUTSEG_PATH/tod_kinect_train_9/ -f $CLUTSEG_PATH/tod_kinect_train_9/config.yaml --verbose -m $mode >> $res
echo "" >> $res
