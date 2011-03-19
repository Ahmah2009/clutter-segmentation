/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <limits>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "cv.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;

/** Tests whether a point cloud file from kinect training data set can be
 * read using routines from 'pcl'. See http://vault.willowgarage.com/wgdata1/vol1/tod_kinect_bags/training */
TEST(PCL, ReadKinectPointCloud) {
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/cloud_00000.pcd", cloud);
}

/** Tests whether a point cloud file from TUM/IAS semantic 3d data set can be
 * read using routines from 'pcl'. Also have a look at the data itself. */
TEST(PCL, ReadSemantic3dPointCloud) {
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);
}

/** Test how points can be projected onto two out of three coordinates. */
TEST(PCL, CoordinateProjection) {
    using namespace cv;
    // TODO: remove platform dependency
    typedef unsigned char uint8;
    PointCloud<PointXYZ> cloud;
    //io::loadPCDFile("./data/sample.delimited.pcd", cloud);
    io::loadPCDFile("/home/julius/Studium/BA/clutter-segmentation/objrec2d/build/semantic-3d/icetea2.delimited.pcd/icetea2_0_.log.delimited.pcd", cloud);

    // find y and z range
    float zmin = numeric_limits<float>::max();
    float zmax = numeric_limits<float>::min();
    float ymin = numeric_limits<float>::max();
    float ymax = numeric_limits<float>::min();
    PointCloud<PointXYZ>::iterator it = cloud.begin();
    PointCloud<PointXYZ>::iterator end = cloud.end();
    while (it != end) {
        zmin = min(zmin, it->z);
        zmax = max(zmax, it->z);
        ymin = min(ymin, it->y);
        ymax = max(ymax, it->y);
        it++;
    }
    // create image and scale of 2d coordinate axes
    float scale = 2000;
    int r = int(scale * (ymax - ymin)) + 1;
    int c = int(scale * (zmax - zmin)) + 1;
    Mat img = Mat::zeros(r, c, CV_8U);
    
    // project points onto image
    it = cloud.begin();
    while (it != end) {
        img.at<uint8>(r - int(scale * (it->y - ymin)) - 1, int(scale * (it->z - zmin))) = 255;
        it++;
    }

    // show image
    namedWindow("TEST(PCL, CoordinateProjection)"); 
    imshow("TEST(PCL, CoordinateProjection)", img);
    waitKey(5000);
}

