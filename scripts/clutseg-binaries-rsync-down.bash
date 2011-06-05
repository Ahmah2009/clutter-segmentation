#!/usr/bin/env bash

function usage() {
cat <<USAGE
Usage: clutseg-binaries-rsync-down
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*
rsync --recursive --archive municware.com:clutseg/ $CLUTSEG_PATH/

trap "exit" INT TERM EXIT

pushd $CLUTSEG_PATH > /dev/null
    libs=$(ssh municware.com "cd clutseg && find . -type d -iname 'lib'")
    bins=$(ssh municware.com "cd clutseg && find . -type d -iname 'bins'")

    for lib in $libs ; do
        printf "Downloading %-60s " $lib
        mkdir -p $lib
        rsync --relative --archive --recursive municware.com:clutseg/$lib/ $lib
        echo "[OK]"
    done

    for bin in $bins ; do
        printf "Downloading %-60s " $bin
        mkdir -p $lib
        rsync --relative --archive --recursive municware.com:clutseg/$bin/ $bin
        echo "[OK]"
    done

popd > /dev/null
