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

pkg_semantic3d_training=$(rospack find semantic3d_training)
testconfig=$(mktemp)
cat >> $testconfig <<EOF
# Write configuration on the fly
[images]
base/tilex/image_00038.png = tilex
base/tilex/image_00016.png = tilex
base/tilex/image_00002.png = tilex
base/silk/image_00038.png = silk
base/silk/image_00016.png = silk
base/silk/image_00002.png = silk
base/tide/image_00038.png = tide
base/tide/image_00016.png = tide
base/tide/image_00002.png = tide
base/fluorescent_paint/image_00038.png = fluorescent_paint
base/fluorescent_paint/image_00016.png = fluorescent_paint
base/fluorescent_paint/image_00002.png = fluorescent_paint
base/teas_tea/image_00038.png = teas_tea
base/teas_tea/image_00016.png = teas_tea
base/teas_tea/image_00002.png = teas_tea
base/campbells_chicken_noodle/image_00038.png = campbells_chicken_noodle
base/campbells_chicken_noodle/image_00016.png = campbells_chicken_noodle
base/campbells_chicken_noodle/image_00002.png = campbells_chicken_noodle
base/fat_free_milk/image_00002.png = fat_free_milk
base/fat_free_milk/image_00011.png = fat_free_milk
base/fat_free_milk/image_00010.png = fat_free_milk
base/downy/image_00002.png = downy
base/downy/image_00011.png = downy
base/downy/image_00010.png = downy
base/odwalla_lime/image_00038.png = odwalla_lime
base/odwalla_lime/image_00016.png = odwalla_lime
base/odwalla_lime/image_00002.png = odwalla_lime
EOF
pushd $CLUTSEG_PATH > /dev/null
    # Collect statistics
    python $pkg_semantic3d_training/scripts/recognizer_stats.py \
           --test $CLUTSEG_PATH \
           --truth $testconfig \
           --base base/ \
           --tod_config base/config.yaml $*
popd > /dev/null
rm $testconfig

