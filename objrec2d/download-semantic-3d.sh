#!/bin/sh
objects=`cat semantic-3d.txt`
mkdir -p build/semantic-3d
cd build/semantic-3d
for obj in $objects
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
done
