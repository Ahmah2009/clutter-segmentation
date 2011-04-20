#!/usr/bin/env bash
if [ "$1" = "--help" ] || [ "$1" = "" ] ; then
    echo "Usage: pose-preprocess <image>"
    exit
fi

convert -monochrome $1 $1.pre.pose.png
