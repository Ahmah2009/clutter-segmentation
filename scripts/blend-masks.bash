#!/usr/bin/env bash
images=$(find . -iname "image_*.png" | grep mask -v | grep pose -v)
for f in $images
do
    if [ -a $f.mask.png ] ; then
        d=$(dirname $f)
        b=$(basename $f)
        mkdir -p $d/blend
        o=$d/blend/$b.mask.blend.png
        echo "processing $f .--> $o"
        composite $f.mask.png $f -blend 60% $o
    fi
done
