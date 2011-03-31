#!/usr/bin/env bash
#

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: frecognizer-debug.bash

Collects statistics using tod kinect training dataset and some of
its images (misused) as training images.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

testconfig=$(mktemp)
cat >> $testconfig <<EOF
# Write configuration on the fly
[images]
tod_kinect_train/tilex/image_00038.png = tilex
tod_kinect_train/tilex/image_00016.png = tilex
tod_kinect_train/tilex/image_00002.png = tilex
tod_kinect_train/silk/image_00038.png = silk
tod_kinect_train/silk/image_00016.png = silk
tod_kinect_train/silk/image_00002.png = silk
tod_kinect_train/tide/image_00038.png = tide
tod_kinect_train/tide/image_00016.png = tide
tod_kinect_train/tide/image_00002.png = tide
tod_kinect_train/fluorescent_paint/image_00038.png = fluorescent_paint
tod_kinect_train/fluorescent_paint/image_00016.png = fluorescent_paint
tod_kinect_train/fluorescent_paint/image_00002.png = fluorescent_paint
tod_kinect_train/teas_tea/image_00038.png = teas_tea
tod_kinect_train/teas_tea/image_00016.png = teas_tea
tod_kinect_train/teas_tea/image_00002.png = teas_tea
tod_kinect_train/campbells_chicken_noodle/image_00038.png = campbells_chicken_noodle
tod_kinect_train/campbells_chicken_noodle/image_00016.png = campbells_chicken_noodle
tod_kinect_train/campbells_chicken_noodle/image_00002.png = campbells_chicken_noodle
tod_kinect_train/fat_free_milk/image_00002.png = fat_free_milk
tod_kinect_train/fat_free_milk/image_00011.png = fat_free_milk
tod_kinect_train/fat_free_milk/image_00010.png = fat_free_milk
tod_kinect_train/downy/image_00002.png = downy
tod_kinect_train/downy/image_00011.png = downy
tod_kinect_train/downy/image_00010.png = downy
tod_kinect_train/odwalla_lime/image_00038.png = odwalla_lime
tod_kinect_train/odwalla_lime/image_00016.png = odwalla_lime
tod_kinect_train/odwalla_lime/image_00002.png = odwalla_lime
EOF
pushd $CLUTSEG_PATH > /dev/null
    # Collect statistics
    python clutter-segmentation/scripts/recognizer_stats.py \
           --test $CLUTSEG_PATH \
           --truth $testconfig \
           --base tod_kinect_train/ \
           --tod_config tod_kinect_train/config.yaml $*
popd > /dev/null
rm $testconfig

