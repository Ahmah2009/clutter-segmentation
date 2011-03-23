/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_candidate/PoseRT.h>

// TODO: move to own namespace
#include "rotation.h"

using namespace pcl;
using namespace pcl_visualization;
using namespace cv;
using namespace opencv_candidate;

TEST(PCL_VISUALIZATION, ShowPointCloud) {
    PointCloud<PointXYZ> cloud;
    PointCloud<PointXYZ> cloud2;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);
    io::loadPCDFile("./data/cloud_00000.pcd", cloud2);
    PCLVisualizer visualizer;
    visualizer.addPointCloud(cloud, "cloud1");
    visualizer.addPointCloud(cloud2, "cloud2");
    visualizer.addCoordinateSystem(1, 0, 0, 0);
    visualizer.spin();
}

TEST(PCL_VISUALIZATION, ShowCircle) {
    PCLVisualizer visualizer;
    PointXYZ origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    visualizer.addSphere(origin, 1, 0, 0, 1, "sphere1");
    visualizer.addSphere(origin, 2, 0, 1, 0, "sphere2");
    visualizer.addSphere(origin, 3, 1, 0, 0, "sphere3");
    visualizer.addCoordinateSystem(1, 0, 0, 0);
    visualizer.spin();
}

TEST(PCL_VISUALIZATION, ShowPoseEstimate) {
    PCLVisualizer visualizer;
    PointXYZ tvec;
    tvec.x = 1;
    tvec.y = -0.25;
    tvec.z = -0.1;
    PointXYZ rvec;
    rvec.x = 1;
    rvec.y = -0.25;
    rvec.z = -0.1;
    PointXYZ origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    visualizer.addCoordinateSystem(1, 0, 0, 0);
    visualizer.addCoordinateSystem(0.2, tvec.x, tvec.y, tvec.z);
    visualizer.addLine(origin, tvec, 0.5, 0.5, 1);
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);
    visualizer.addPointCloud(cloud);
    visualizer.spin();
}

// FIXME: rvec must probably somehow fit to Rodrigues 3d rotation formula
// I guess ||rvec|| is the angle, and rvec is the axis of rotation
TEST(PCL_VISUALIZATION, ShowFiducialPoseEstimate) {
    // Load pose estimation from yaml file
    FileStorage fs("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    PoseRT pose;
    pose.read(fs[PoseRT::YAML_NODE_NAME]);
    // Load point cloud
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
    // Create visualizer
    PCLVisualizer visualizer;
    // Create rotation matrix according to pose.rvec
    // TODO: is pose.rvec really to be interpreted like this???
    Mat rx = Mat(3, 3, CV_64F);
    Mat ry = Mat(3, 3, CV_64F);
    Mat rz = Mat(3, 3, CV_64F);
    Mat rot = Mat(3, 3, CV_64F);
    populateXRotationMatrix(rx, pose.rvec.at<double>(0, 0));
    populateYRotationMatrix(ry, pose.rvec.at<double>(1, 0));
    populateZRotationMatrix(rz, pose.rvec.at<double>(2, 0));
    rot = rz*ry*rx;
    // rot = rx*ry*rz;
    // Model unit vectors along coordinate axes
    // and rotate and translate them
    Mat xunit = Mat(3, 1, CV_64F);
    xunit.at<double>(0, 0) = 1;
    xunit.at<double>(1, 0) = 0;
    xunit.at<double>(2, 0) = 0;
    xunit = rot * xunit + pose.tvec;
    Mat yunit = Mat(3, 1, CV_64F);
    yunit.at<double>(0, 0) = 0;
    yunit.at<double>(1, 0) = 1;
    yunit.at<double>(2, 0) = 0;
    yunit = rot * yunit + pose.tvec;
    Mat zunit = Mat(3, 1, CV_64F);
    zunit.at<double>(0, 0) = 0;
    zunit.at<double>(1, 0) = 0;
    zunit.at<double>(2, 0) = 1;
    zunit = rot * zunit + pose.tvec;

    cout << "==================================================" << endl;
    cout << pose.tvec.at<double>(0, 0) << endl;
    cout << pose.tvec.at<double>(1, 0) << endl;
    cout << pose.tvec.at<double>(2, 0) << endl;

    PointXYZ origin;
    // the tip of the x-axis drawn in the pose
    PointXYZ xtip;
    xtip.x = xunit.at<double>(0, 0);
    xtip.y = xunit.at<double>(1, 0);
    xtip.z = xunit.at<double>(2, 0);
    // the tip of the y-axis drawn in the pose
    PointXYZ ytip;
    ytip.x = yunit.at<double>(0, 0);
    ytip.y = yunit.at<double>(1, 0);
    ytip.z = yunit.at<double>(2, 0);
    // the tip of the z-axis drawn in the pose
    PointXYZ ztip;
    ztip.x = zunit.at<double>(0, 0);
    ztip.y = zunit.at<double>(1, 0);
    ztip.z = zunit.at<double>(2, 0);
    // Just pose.tvec as PointXYZ
    PointXYZ tvec;
    tvec.x = pose.tvec.at<double>(0, 0);
    tvec.y = pose.tvec.at<double>(1, 0);
    tvec.z = pose.tvec.at<double>(2, 0);
    // Add coordinate system
    visualizer.addCoordinateSystem(1, 0, 0, 0);
    // Draw pose
    visualizer.addLine(tvec, xtip, 1, 0, 0, "tvec-xtip");
    visualizer.addLine(tvec, ytip, 0, 1, 0, "tvec-ytip");
    visualizer.addLine(tvec, ztip, 0, 0, 1, "tvec-ztip");
    // Add point cloud
    visualizer.addPointCloud(cloud);
    visualizer.spin();
}

