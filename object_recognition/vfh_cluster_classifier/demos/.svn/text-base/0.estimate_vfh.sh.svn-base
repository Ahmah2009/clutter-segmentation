#!/bin/bash
# Example directory containing the PCD files that we need to process. Assume 1 cluster / file.
DATA=`pwd`/data

# Fire up the launch file
#roslaunch vfh_cluster_classifier estimate_vfh.launch

# Set PCL nodelet parameters
rosparam set /Reader/filename ""

# Parse all files
for i in `find $DATA -type f \( -iname "*cluster*nxyz.pcd" ! -iname "*vfh*" \)`
do
  echo "Processing $i"
  base=`echo $i | awk -F ".pcd" {'print $1'}`

  # Remove old files
  rm -f $base"_vfh.pcd"

  rosparam set /WriterVFH/filename $base"_vfh.pcd"

  # Set the PCDReader nodelet parameter
  rosparam set /Reader/tf_frame $i
  rosparam set /Reader/filename $i

  # Wait until files were written to disk
  while [ ! -e $base"_vfh.pcd" ]; do sleep 0.1; done 
done
