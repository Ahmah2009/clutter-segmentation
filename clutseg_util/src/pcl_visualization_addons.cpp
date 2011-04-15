/**
 * Author: Julius Adorf
 */

#include "pcl_visualization_addons.h"

#include <opencv_candidate/PoseRT.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <cv.h>
#include <assert.h>

using namespace pcl;
using namespace pcl_visualization;
using namespace opencv_candidate;
using namespace cv;

void addPose(PCLVisualizer & visualizer, PoseRT & pose, string id_prefix) {
    int axlength = 1;

    Mat rot;
    Rodrigues(pose.rvec, rot);
    // Unit vectors, rotated and translated according to pose
    Mat xpose(3, 1, CV_64FC1);
    xpose.at<double>(0, 0) = axlength;
    xpose.at<double>(1, 0) = 0;
    xpose.at<double>(2, 0) = 0;
    xpose = rot * xpose;
    xpose = xpose + pose.tvec;
    Mat ypose = Mat(3, 1, CV_64FC1);
    ypose.at<double>(0, 0) = 0;
    ypose.at<double>(1, 0) = axlength;
    ypose.at<double>(2, 0) = 0;
    ypose = rot * ypose + pose.tvec;
    Mat zpose = Mat(3, 1, CV_64FC1);
    zpose.at<double>(0, 0) = 0;
    zpose.at<double>(1, 0) = 0;
    zpose.at<double>(2, 0) = axlength;
    zpose = rot * zpose + pose.tvec;
    // the tip of the x-axis drawn in the pose
    PointXYZ xtip;
    xtip.x = xpose.at<double>(0, 0);
    xtip.y = xpose.at<double>(1, 0);
    xtip.z = xpose.at<double>(2, 0);
    // the tip of the y-axis drawn in the pose
    PointXYZ ytip;
    ytip.x = ypose.at<double>(0, 0);
    ytip.y = ypose.at<double>(1, 0);
    ytip.z = ypose.at<double>(2, 0);
    // the tip of the z-axis drawn in the pose
    PointXYZ ztip;
    ztip.x = zpose.at<double>(0, 0);
    ztip.y = zpose.at<double>(1, 0);
    ztip.z = zpose.at<double>(2, 0);
    // Just pose.tvec as PointXYZ
    PointXYZ tvec;
    tvec.x = pose.tvec.at<double>(0, 0);
    tvec.y = pose.tvec.at<double>(1, 0);
    tvec.z = pose.tvec.at<double>(2, 0);
    // Draw pose
    visualizer.addLine(tvec, xtip, 1, 0, 0, id_prefix + "tvec-xtip");
    visualizer.addLine(tvec, ytip, 0, 1, 0, id_prefix + "tvec-ytip");
    visualizer.addLine(tvec, ztip, 0, 0, 1, id_prefix + "tvec-ztip");
}

