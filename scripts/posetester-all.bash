#!/usr/bin/env bash
#
#

images=$(find . -iname "*.png" | grep -v "mask" | grep -v "pose" )
posetester $images

