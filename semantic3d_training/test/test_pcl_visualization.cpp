/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_visualization/pcl_visualizer.h"
#include "cv.h"
#include <opencv2/highgui/highgui.hpp>

using namespace pcl;
using namespace pcl_visualization;

TEST(PCL_VISUALIZATION, ShowPointCloud) {
    PointCloud<PointXYZ> cloud;
    PointCloud<PointXYZ> cloud2;
    io::loadPCDFile("./data/sample.delimited.pcd", cloud);
    io::loadPCDFile("./data/cloud_00000.pcd", cloud2);
    PCLVisualizer visualizer;
    visualizer.addPointCloud(cloud, "cloud1");
    visualizer.addPointCloud(cloud2, "cloud2");
    visualizer.addCoordinateSystem(1, 0, 0, 0);
    visualizer.spin(10000);
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
    visualizer.spin(10000);
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

