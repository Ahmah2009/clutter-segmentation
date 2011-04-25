#!/usr/bin/env bash

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

flag_help=
flag_undo=

while getopts 'hu' OPTION
do
    case $OPTION in
    h)  flag_help=1
        ;;
    u)  flag_undo=1
        ;;
    esac
done

shift $(($OPTIND-1))
testbase=$CLUTSEG_PATH/$1

if [ "$flag_help" = "1" ] || [ "$testbase" = "" ] ; then
    cat <<HELP
Usage: link-test-results <test-base>

Creates folders with symlinks to selected artifacts in a
test result directory.
HELP
    exit
fi

function create_symlinks() {
    pushd $1 > /dev/null
       echo "Creating symlinks $1 ..."
       ls -1 $testbase/result/*$2 | xargs -l ln -f -s 
    popd > /dev/null
}

function montage_all() {
    pushd $1 > /dev/null
       echo "Imagemagick montage $1 ..."
       rm -f $1.png
       montage *.png -geometry +10+10 $1.png
    popd > /dev/null
}

pushd $testbase/result > /dev/null
    if [ "$flag_undo" ]; then
        rm -r guesses-all
        rm -r guesses-single
        rm -r matches-merged
        rm -r matches-all
        rm -r projection
        rm -r keypoints 
    else
        mkdir -p guesses-all
        mkdir -p guesses-single
        mkdir -p matches-merged
        mkdir -p matches-all
        mkdir -p projection
        mkdir -p keypoints 

        create_symlinks guesses-all .guesses.png
        # montage_all guesses-all
        
        create_symlinks guesses-single .guess.png
        # montage_all guesses-single

        create_symlinks matches-merged .merged.matches.png
        # montage_all matches-merged

        create_symlinks projection .projection.png
        # montage_all projection

        create_symlinks keypoints .keypoints.png
        # montage_all keypoints

        create_symlinks matches-all .matches.png
        # TODO: fixme
        rm $testbase/result/matches-all/*.merged.matches.png
    fi
popd > /dev/null

