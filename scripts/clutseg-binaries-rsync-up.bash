#!/usr/bin/env bash

function usage() {
cat <<USAGE
Usage: clutseg-binaries-rsync-up
USAGE
}

source ~/.profile
source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

trap "exit" INT TERM EXIT

pushd $CLUTSEG_PATH > /dev/null
    libs=$(find . -type d -iname "lib")
    bins=$(find . -type d -iname "bin")

    for lib in $libs ; do
        printf "Uploading %-60s " $lib
        rsync --relative --archive --recursive --delete $lib municware.com:clutseg/
        echo "[OK]"
    done

    for bin in $bins ; do
        printf "Uploading %-60s " $bin
        rsync --relative --archive --recursive --delete $bin municware.com:clutseg/
        echo "[OK]"
    done
popd > /dev/null
