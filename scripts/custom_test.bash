#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: custom_test.bash

Recognizes objects in custom test folder, using tod kinect training database.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

res=$CLUTSEG_PATH/custom_test/results.txt
echo "Experiment running on "`date` >> $res
echo "--------------------------------------------------" >> $res
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "config.yaml" >> $res
echo "--------------------------------------------------" >> $res
cat $CLUTSEG_PATH/base/config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res
echo "features.config.yaml" >> $res
echo "--------------------------------------------------" >> $res
cat $CLUTSEG_PATH/base/features.config.yaml >> $res 
echo "--------------------------------------------------" >> $res
echo "" >> $res
echo "" >> $res

for pic in $CLUTSEG_PATH/custom_test/*.png ; do
    echo $pic >> $res
    rosrun tod_detecting recognizer -I $pic -B  $CLUTSEG_PATH/base/ -f $CLUTSEG_PATH/base/config.yaml -V 0 -m 1 | grep "imageId" >> $res
    echo "" >> $res
done
