/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <limits>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "cv.h"
#include <opencv2/highgui/highgui.hpp>

#include "fiducial/fiducial.h"
#include "tod/core/Features3d.h"
#include "tod/training/masking.h"
#include "tod/training/clouds.h"
#include "tod/training/Opts.h"
#include "opencv_candidate/Camera.h"


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
TEST(PCL, ReadSemantic3dPointCloudAsXYZ) {
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);
}

/** Demonstrates how a point cloud file from TUM/IAS semantic 3d data set can NOT be
* read using routines from 'pcl'. Also have a look at the data itself. */
TEST(PCL, BadReadSemantic3dPointCloudAsXYZRGB) {
    PointCloud<PointXYZRGB> cloud;
    ASSERT_THROW(io::loadPCDFile("./data/sample.delimited.pcd", cloud), InvalidConversionException);
}

/** Tests whether a point cloud file from TUM/IAS semantic 3d data set can be
 * read using routines from 'pcl'. Also have a look at the data itself. */
TEST(PCL, GoodReadSemantic3dPointCloudAsXYZRGB) {
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/sample.delimited.rgb.pcd", cloud);
}

/** Tests how to convert xyz point cloud data to xyzrgb point cloud data. */
TEST(PCL, ConvertXYZtoXYZRGB) {
    PointCloud<PointXYZ> cloud_xyz;
    PointCloud<PointXYZRGB> cloud_xyzrgb;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud_xyz);
    cloud_xyzrgb.points.resize(cloud_xyz.size());
    for (unsigned int i = 0; i < cloud_xyz.points.size(); i++) {
        cloud_xyzrgb.points[i].x = cloud_xyz.points[i].x;
        cloud_xyzrgb.points[i].y = cloud_xyz.points[i].y;
        cloud_xyzrgb.points[i].z = cloud_xyz.points[i].z;
    }
    io::savePCDFileASCII("build/sample.delimited.xyzrgb.pcd", cloud_xyzrgb);
    PointCloud<PointXYZRGB> cloud_xyzrgb_2;
    io::loadPCDFile("build/sample.delimited.xyzrgb.pcd", cloud_xyzrgb_2);
    for (unsigned int i = 0; i < cloud_xyzrgb.points.size(); i++) {
        ASSERT_EQ(cloud_xyzrgb.points[i].x, cloud_xyzrgb_2.points[i].x);
        ASSERT_EQ(cloud_xyzrgb.points[i].y, cloud_xyzrgb_2.points[i].y);
        ASSERT_EQ(cloud_xyzrgb.points[i].z, cloud_xyzrgb_2.points[i].z);
    }
}

/** Test how points can be projected onto two out of three coordinates. Use the
 * characteristically shaped icetea2 object and project the y and z coordinates
 * onto a picture. */
TEST(PCL, CoordinateProjection) {
    using namespace cv;
    // TODO: remove platform dependency
    typedef unsigned char uint8;
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);

    // find y and z range
    float ymin = numeric_limits<float>::max();
    float ymax = numeric_limits<float>::min();
    float zmin = numeric_limits<float>::max();
    float zmax = numeric_limits<float>::min();
    PointCloud<PointXYZ>::iterator it = cloud.begin();
    PointCloud<PointXYZ>::iterator end = cloud.end();
    while (it != end) {
        ymin = min(ymin, it->y);
        ymax = max(ymax, it->y);
        zmin = min(zmin, it->z);
        zmax = max(zmax, it->z);
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
    imshow("TEST(PCL, CoordinateProjection)", img);
    waitKey(5000);
}

/** Test how to create a mask by point cloud segmentation / perspective projection */
TEST(PCL, PerspectiveProjection) {
    // See also masker.cpp in tod_training
    fiducial::KnownPoseEstimator pose_est("./data/fat_free_milk_image_00000.png.pose.yaml");
    cv::Mat colorimg = cv::imread("./data/fat_free_milk_image_00000.png", CV_LOAD_IMAGE_COLOR);
    tod::Camera camera = tod::Camera("./data/fat_free_milk_camera.yml", opencv_candidate::Camera::TOD_YAML);
    tod::Features2d f2d(camera, colorimg);
    f2d.camera.pose = pose_est.estimatePose(cv::Mat());
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
    f2d.mask = tod::cloudMask(cloud, f2d.camera.pose, camera);
    cv::Mat colorMask;
    cv::cvtColor(f2d.mask, colorMask, CV_GRAY2BGR);
    cv::imshow("TEST(PCL, PerspectiveProjection)", f2d.image & colorMask);
    cv::waitKey(5000);
}

/** Test how to fill in pose information without loading it from a file */
TEST(PCL, PerspectiveProjectionManualPose) {
    // See also masker.cpp in tod_training
    cv::Mat colorimg = cv::imread("./data/fat_free_milk_image_00000.png", CV_LOAD_IMAGE_COLOR);
    tod::Camera camera = tod::Camera("./data/fat_free_milk_camera.yml", opencv_candidate::Camera::TOD_YAML);
    tod::Features2d f2d(camera, colorimg);
    // Approximately fill in rotation and translation vectors as can be found
    // in ./data/fat_free_milk_image_00000.png.pose.yaml
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    rvec.at<float>(0, 0) = 2.1;
    rvec.at<float>(1, 0) = 0.52;
    rvec.at<float>(2, 0) = -0.25;
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    tvec.at<float>(0, 0) = 0.056;
    tvec.at<float>(1, 0) = 0.052;
    tvec.at<float>(2, 0) = 0.81;
    f2d.camera.pose.rvec = rvec; 
    f2d.camera.pose.tvec = tvec;
    // Load point cloud and perform segmentation and perspective projection
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
    f2d.mask = tod::cloudMask(cloud, f2d.camera.pose, camera);
    cv::Mat colorMask;
    cv::cvtColor(f2d.mask, colorMask, CV_GRAY2BGR);
    cv::imshow("TEST(PCL, PerspectiveProjectionManualPose)", f2d.image & colorMask);
    cv::waitKey(5000);
}

/** Test how distorted pose estimation value affects masking process */
TEST(PCL, PerspectiveProjectionDistortedPose) {
    // See also masker.cpp in tod_training
    cv::Mat colorimg = cv::imread("./data/fat_free_milk_image_00000.png", CV_LOAD_IMAGE_COLOR);
    tod::Camera camera = tod::Camera("./data/fat_free_milk_camera.yml", opencv_candidate::Camera::TOD_YAML);
    tod::Features2d f2d(camera, colorimg);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    rvec.at<float>(0, 0) = 2.4;
    rvec.at<float>(1, 0) = 0.83;
    rvec.at<float>(2, 0) = -0.56;
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    tvec.at<float>(0, 0) = 0.056;
    tvec.at<float>(1, 0) = 0.052;
    tvec.at<float>(2, 0) = 0.81;
    f2d.camera.pose.rvec = rvec; 
    f2d.camera.pose.tvec = tvec;
    // Load point cloud and perform segmentation and perspective projection
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
    f2d.mask = tod::cloudMask(cloud, f2d.camera.pose, camera);
    cv::Mat colorMask;
    cv::cvtColor(f2d.mask, colorMask, CV_GRAY2BGR);
    cv::imshow("TEST(PCL, PerspectiveProjectionDistortedPose)", f2d.image & colorMask);
    cv::waitKey(5000);
}

/** Manually estimated pose by using pcl_visualization */
TEST(PCL, PerspectiveProjectionManualPose2) {
    // See also masker.cpp in tod_training
    cv::Mat colorimg = cv::imread("./data/icetea2_00000.png", CV_LOAD_IMAGE_COLOR);
    tod::Camera camera = tod::Camera("./data/fat_free_milk_camera.yml", opencv_candidate::Camera::TOD_YAML);
    tod::Features2d f2d(camera, colorimg);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    rvec.at<float>(0, 0) = 0;
    rvec.at<float>(1, 0) = 0;
    rvec.at<float>(2, 0) = 0;
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1); 
    tvec.at<float>(0, 0) = 1;
    tvec.at<float>(1, 0) = -0.25;
    tvec.at<float>(2, 0) = -0.1;
    f2d.camera.pose.rvec = rvec; 
    f2d.camera.pose.tvec = tvec;
    // Load point cloud and perform segmentation and perspective projection
    PointCloud<PointXYZRGB> cloud;
    io::loadPCDFile("./data/cloud_00000.pcd", cloud);
    f2d.mask = tod::cloudMask(cloud, f2d.camera.pose, camera);
    cv::Mat colorMask;
    cv::cvtColor(f2d.mask, colorMask, CV_GRAY2BGR);
    cv::imshow("TEST(PCL, PerspectiveProjectionManualPose2)", f2d.image & colorMask);
    cv::waitKey(5000);
}

