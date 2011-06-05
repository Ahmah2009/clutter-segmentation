#!/usr/bin/env bash

# Note: 2011-03-31: works
# Note: 2011-04-04: works

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: custom_test.bash

Recognizes objects in custom test folder, using tod classifier trained on tod
kinect dataset.
HELP
    exit
fi

source ~/.env
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

res=$CLUTSEG_PATH/custom_test/results.txt
echo "Experiment running on "`date` >> $res
echo "--------------------------------------------------" >> $res
echo "--------------------------------------------------" >> $res
echo "Using tod kinect training set 9 and custom testing set" >> $res
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

for pic in $CLUTSEG_PATH/custom_test/*.png ; do
    echo $pic >> $res
    rosrun tod_detecting recognizer -I $pic -B  $CLUTSEG_PATH/tod_kinect_train_9/ -f $CLUTSEG_PATH/tod_kinect_train_9/config.yaml -V 0 -m 1 | grep "imageId" >> $res
    echo "" >> $res
done
