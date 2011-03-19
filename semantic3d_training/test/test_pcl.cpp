/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

/** Tests whether a point cloud file from kinect training data set can be
 * read using routines from 'pcl'. See http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training */
TEST(PCL, ReadKinectPointCloud) {
    using namespace pcl;
    PointCloud<PointXYZRGB> pcl_cloud;
    io::loadPCDFile("./data/cloud_00000.pcd", pcl_cloud);
}

/** Tests whether a point cloud file from TUM/IAS semantic 3d data set can be
 * read using routines from 'pcl'. Also have a look at the data itself. */
TEST(PCL, ReadSemantic3dPointCloud) {
    using namespace pcl;
    PointCloud<PointXYZ> pcl_cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", pcl_cloud);
}

/** Test how points can be projected onto one of the coordinates. */
TEST(PCL, CoordinateProjection) {
    using namespace pcl;
    PointCloud<PointXYZ> pcl_cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", pcl_cloud);
}

