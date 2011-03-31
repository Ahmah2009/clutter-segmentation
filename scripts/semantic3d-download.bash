#!/usr/bin/env bash

if [ "$1" = "--help" ] || [ ! "$1" ] ; then
    cat <<HELP
Usage: semantic3d-download.bash [--no-extract] <directory>

Downloads all semantic dataset from IAS/TUM using wget to any specified
directory.
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi

objlist=$(mktemp)

cat >> $objlist <<EOF
bean-can
book1-robotic-manipulation
book2-machine-learning
book3-opencv
chopping-board
coffee-filter-big
coffee-filter-small
fizzy-tablet
icetea
icetea2
ikea-blue-bowl
instant-soup
juice-tetrapak
kitchen-roll-new
kitchen-roll-used
milk-tetrapak
mug1
mug3
mug5
nivea-creme
nivea-shampoo
nivea-suncreme
pepper
plate1
plate2
plate3
salt
tape
tea-lancaster
tea-messner
white-bowl
yellow-pan
yoghurt
EOF

mkdir -p $1
pushd $1 > /dev/null
    for obj in $(cat $objlist)
    do
        echo "Cleaning up $obj ..."
        rm -f $obj.delimited.pcd.tar.bz2
        rm -f $obj.delimited.rotated.pcd.tar.bz2
        rm -f $obj.pcd.tar.bz2
        rm -f $obj.tar.bz2
        rm -f $obj.png.tar.bz2

        rm -rf $obj.delimited.pcd
        rm -rf $obj.delimited.rotated.pcd
        rm -rf $obj.pcd
        rm -rf $obj
        rm -rf $obj.png

        echo "Downloading $obj ..."
        wget -nv http://ias.cs.tum.edu/download/semantic-3d/data/pcds-delimited/$obj.delimited.pcd.tar.bz2
        wget -nv http://ias.cs.tum.edu/download/semantic-3d/data/pcds-rotated/$obj.delimited.rotated.pcd.tar.bz2
        wget -nv http://ias.cs.tum.edu/download/semantic-3d/data/pcds-full/$obj.pcd.tar.bz2
        wget -nv http://ias.cs.tum.edu/download/semantic-3d/data/raw/$obj.tar.bz2
        wget -nv http://ias.cs.tum.edu/download/semantic-3d/data/images-roi/$obj.png.tar.bz2

        if [ ! "$1" = "--no-extract" ] && [ ! "$2" = "--no-extract" ] ; then
            echo "Creating directories for $obj ..."
            mkdir $obj.delimited.pcd
            mkdir $obj.delimited.rotated.pcd
            mkdir $obj.pcd
            mkdir $obj
            mkdir $obj.png

            echo "Extracting files for $obj ..."
            tar -x -C $obj.delimited.pcd           -f $obj.delimited.pcd.tar.bz2
            tar -x -C $obj.delimited.rotated.pcd   -f $obj.delimited.rotated.pcd.tar.bz2
            tar -x -C $obj.pcd                     -f $obj.pcd.tar.bz2
            tar -x -C $obj                         -f $obj.tar.bz2 
            tar -x -C $obj.png                     -f $obj.png.tar.bz2
        fi
    done
popd > /dev/null
rm $objlist

