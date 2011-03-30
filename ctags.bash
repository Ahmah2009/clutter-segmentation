#!/usr/bin/env bash
# Creates tags for diamondback sources, clutter-segmentation, stack
# object_recognition, and dependencies of tod_training and tod_detecting.
if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi
ctags --recurse /opt/ros/diamondback $CLUTSEG_PATH 

