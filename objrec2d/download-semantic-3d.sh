#!/bin/sh
objects=`cat semantic-3d.txt`
for obj in $objects
do
    echo "Downloading $obj ..."
    wget http://ias.cs.tum.edu/download/semantic-3d/data/pcds-delimited/$obj.delimited.pcd.tar.bz2
    wget http://ias.cs.tum.edu/download/semantic-3d/data/pcds-rotated/$obj.delimited.rotated.pcd.tar.bz2
    wget http://ias.cs.tum.edu/download/semantic-3d/data/pcds-full/$obj.pcd.tar.bz2
    wget http://ias.cs.tum.edu/download/semantic-3d/data/raw/$obj.tar.bz2
    wget http://ias.cs.tum.edu/download/semantic-3d/data/images-roi/$obj.png.tar.bz2
done
