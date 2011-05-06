#!/usr/bin/env bash
for mod in $(cat $CLUTSEG_PATH/clutter-segmentation/mods/mods.txt) ; do
    src=$CLUTSEG_PATH/clutter-segmentation/mods/$mod
    dst=$CLUTSEG_PATH/$mod 
    if [ -f $dst ] ; then
        echo "Skipping $mod, destination exists."
    else
        ln -v $src $dst
    fi
done

