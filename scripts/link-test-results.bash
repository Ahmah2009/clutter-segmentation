#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: link-test-results <test-base>

Creates folders with symlinks to selected artifacts in a
test result directory.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg $1 

testbase=$CLUTSEG_PATH/$1


# create_symlinks <folder-name> <glob-suffix-pattern>
function create_symlinks() {
    pushd $testbase/result > /dev/null
        mkdir -p $1
        pushd $1 > /dev/null
           echo "Creating symlinks $1 ..."
           ls -1 $testbase/result/*$2 | xargs -l ln -f -s 
        popd > /dev/null
    popd > /dev/null
}

create_symlinks     guesses-all     .guesses.png
create_symlinks     guesses-single  .guess.png
create_symlinks     matches-merged  .merged.matches.png
create_symlinks     matches-all     .all.matches.png
create_symlinks     projection      .projection.png
create_symlinks     keypoints       .keypoints.png

