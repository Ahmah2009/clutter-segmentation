#!/usr/bin/env bash

if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: download-tod-kinect-bags.bash <bagsdir>

Downloads tod kinect training bags from
http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

# interesting 
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/multiview.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/multiview.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/suave3in1.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/suave3in1.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/opencv.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/opencv.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/spam.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/spam.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/good_earth_tea.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/good_earth_tea.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/claritin_d.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/claritin_d.tf.bag

# not as interesting
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/arm_hammer.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/arm_hammer.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_chicken_noodle.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_chicken_noodle.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_creamy_tomato.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/campbells_creamy_tomato.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/cascade.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/cascade.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/clearasil.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/clearasil.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffee_filter.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffee_filter.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffeemate.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/coffeemate.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/contact_solution.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/contact_solution.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/country_crock.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/country_crock.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/crest.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/crest.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/delmonte_peas_carrots.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/delmonte_peas_carrots.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/dove.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/dove.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fat_free_milk.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fat_free_milk.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fluorescent_paint.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/fluorescent_paint.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/egg_poacher.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/egg_poacher.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/expo_dry_erase.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/expo_dry_erase.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/gillette.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/gillette.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/happy_cup.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/happy_cup.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/hersheys.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/hersheys.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/invisible_tape.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/invisible_tape.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/izzy.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/izzy.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/japenese_tea.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/japenese_tea.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/kinect_box.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/kinect_box.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/mopandglow.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/mopandglow.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/oatmeal_crisp.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/oatmeal_crisp.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_lime.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_lime.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_orange.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_orange.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_pome.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/odwalla_pome.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/pantene.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/pantene.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/playing_cards.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/playing_cards.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/raisin_bran.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/raisin_bran.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/rishitea.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/rishitea.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/robot_power.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/robot_power.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/silk.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/silk.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/sterile_pads.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/sterile_pads.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tazo_chai.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tazo_chai.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tropicana.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tropicana.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/teas_tea.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/teas_tea.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tide.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tide.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tilex.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/tilex.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/unicorn_plate.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/unicorn_plate.tf.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/wine_opener.bag.bag
wget -P $1 http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training/wine_opener.tf.bag

