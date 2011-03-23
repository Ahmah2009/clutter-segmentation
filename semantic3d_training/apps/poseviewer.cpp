/**
 * Author: Julius Adorf
 */

#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv_candidate/PoseRT.h>
#include <cv.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl_visualization_addons.h"

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace pcl_visualization;

int main(int argc, char *argv[]) {
    // Read arguments
    if (argc != 8) {
        cerr << "Usage: poseviewer <point-cloud-file> <tvec.x> <tvec.y> <tvec.z> <rvec.x> <rvec.y> <rvec.z>" << endl;
        return 1;
    }
    string pcd_file(argv[1]);
    PoseRT pose;
    pose.tvec = Mat(3, 1, CV_64F);
    pose.rvec = Mat(3, 1, CV_64F);
    pose.tvec.at<double>(0, 0) = boost::lexical_cast<double>(argv[2]);
    pose.tvec.at<double>(1, 0) = boost::lexical_cast<double>(argv[3]);
    pose.tvec.at<double>(2, 0) = boost::lexical_cast<double>(argv[4]);
    pose.rvec.at<double>(0, 0) = boost::lexical_cast<double>(argv[5]);
    pose.rvec.at<double>(1, 0) = boost::lexical_cast<double>(argv[6]);
    pose.rvec.at<double>(2, 0) = boost::lexical_cast<double>(argv[7]);

    // Repeat arguments for easier reference
    cout << boost::format("Point cloud file:   %s\n") % pcd_file; 
    cout << boost::format("Translation vector: %5.3f %5.3f %5.3f\n") % pose.tvec.at<double>(0, 0) % pose.tvec.at<double>(1, 0) % pose.tvec.at<double>(2, 0); 
    cout << boost::format("Rotation vector:    %5.3f %5.3f %5.3f\n") % pose.rvec.at<double>(0, 0) % pose.rvec.at<double>(1, 0) % pose.rvec.at<double>(2, 0); 

    // Load point cloud
    PointCloud<PointXYZ> cloud;
    io::loadPCDFile(pcd_file, cloud);
    
    // Create visualization
    PCLVisualizer visualizer;
    visualizer.addCoordinateSystem(0.5, 0, 0, 0);
    visualizer.addPointCloud(cloud);
    addPose(visualizer, pose);
    
    // show visualization
    visualizer.spin(); 
}

