#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: download-tod-kinect-bags.bash

Downloads tod kinect training bags from
http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

pushd $CLUTSEG_PATH/tod_kinect_bags > /dev/null
    # interesting 
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/multiview.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/multiview.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/suave3in1.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/suave3in1.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/opencv.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/opencv.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/spam.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/spam.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/good_earth_tea.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/good_earth_tea.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/claritin_d.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/claritin_d.tf.bag

    # not as interesting
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/arm_hammer.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/arm_hammer.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_chicken_noodle.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_chicken_noodle.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_creamy_tomato.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_creamy_tomato.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/cascade.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/cascade.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/clearasil.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/clearasil.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffee_filter.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffee_filter.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffeemate.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffeemate.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/contact_solution.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/contact_solution.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/country_crock.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/country_crock.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/crest.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/crest.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/delmonte_peas_carrots.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/delmonte_peas_carrots.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/dove.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/dove.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fat_free_milk.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fat_free_milk.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fluorescent_paint.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fluorescent_paint.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/egg_poacher.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/egg_poacher.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/expo_dry_erase.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/expo_dry_erase.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/gillette.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/gillette.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/happy_cup.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/happy_cup.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/hersheys.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/hersheys.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/invisible_tape.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/invisible_tape.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/izzy.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/izzy.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/japenese_tea.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/japenese_tea.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/kinect_box.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/kinect_box.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/mopandglow.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/mopandglow.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/oatmeal_crisp.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/oatmeal_crisp.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_lime.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_lime.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_orange.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_orange.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_pome.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_pome.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/pantene.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/pantene.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/playing_cards.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/playing_cards.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/raisin_bran.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/raisin_bran.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/rishitea.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/rishitea.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/robot_power.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/robot_power.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/silk.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/silk.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/sterile_pads.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/sterile_pads.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tazo_chai.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tazo_chai.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tropicana.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tropicana.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/teas_tea.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/teas_tea.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tide.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tide.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tilex.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tilex.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/unicorn_plate.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/unicorn_plate.tf.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/wine_opener.bag.bag
    wget http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/wine_opener.tf.bag

popd > /dev/null
