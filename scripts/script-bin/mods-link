#!/usr/bin/env bash
source ~/.env
for mod in $(cat $CLUTSEG_PATH/clutter-segmentation/mods/mods.txt) ; do
    src=$CLUTSEG_PATH/clutter-segmentation/mods/$mod
    dst=$CLUTSEG_PATH/$mod 
    ln -v -f $src $dst
done

