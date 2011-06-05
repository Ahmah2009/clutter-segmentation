#!/usr/bin/env bash

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*
rsync -avP municware.com:clutseg/* $CLUTSEG_PATH/
