#!/usr/bin/env bash

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

pushd $CLUTSEG_PATH > /dev/null
    libs=$(find . -type d -iname "lib")
    bins=$(find . -type d -iname "bin")

    for lib in $libs ; do
        ssh municware.com "mkdir -p clutseg/$lib"
        rsync -avP --delete $lib/* municware.com:clutseg/$lib
    done

    for bin in $bins ; do
        ssh municware.com "mkdir -p clutseg/$bin"
        rsync -ravP --delete $bin/* municware.com:clutseg/$bin
    done
popd > /dev/null
