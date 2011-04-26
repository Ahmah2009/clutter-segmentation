/**
 * Author: Julius Adorf
 */

#include "clutseg/pcl_visualization_addons.h"

#include <opencv_candidate/PoseRT.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <cv.h>
#include <assert.h>
#include <boost/format.hpp>

using namespace pcl;
using namespace pcl_visualization;
using namespace opencv_candidate;
using namespace cv;

namespace clutseg {

    void addPose(PCLVisualizer & visualizer, const PoseRT & pose, const string & id_prefix) {
        int axlength = 1;

        Mat rot;
        Rodrigues(pose.rvec, rot);
        rot.convertTo(rot, CV_64FC1);
        Mat t;
        pose.tvec.convertTo(t, CV_64FC1);

        // Unit vectors, rotated and translated according to pose
        Mat xpose(3, 1, CV_64FC1);
        xpose.at<double>(0, 0) = axlength;
        xpose.at<double>(1, 0) = 0;
        xpose.at<double>(2, 0) = 0;
        xpose = rot * xpose;
        xpose = xpose + t;
        Mat ypose = Mat(3, 1, CV_64FC1);
        ypose.at<double>(0, 0) = 0;
        ypose.at<double>(1, 0) = axlength;
        ypose.at<double>(2, 0) = 0;
        ypose = rot * ypose + t;
        Mat zpose = Mat(3, 1, CV_64FC1);
        zpose.at<double>(0, 0) = 0;
        zpose.at<double>(1, 0) = 0;
        zpose.at<double>(2, 0) = axlength;
        zpose = rot * zpose + t;
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
        tvec.x = t.at<double>(0, 0);
        tvec.y = t.at<double>(1, 0);
        tvec.z = t.at<double>(2, 0);
        // Draw pose
        visualizer.addLine(tvec, xtip, 1, 0, 0, id_prefix + "tvec-xtip");
        visualizer.addLine(tvec, ytip, 0, 1, 0, id_prefix + "tvec-ytip");
        visualizer.addLine(tvec, ztip, 0, 0, 1, id_prefix + "tvec-ztip");
    }

    void addMarker3d(PCLVisualizer & visualizer, const PointXYZ & p, const string & id_prefix) {
        visualizer.addSphere(p, 0.01, 1.0, 0.0, 0.0, id_prefix + "sphere");
        visualizer.addText(id_prefix, p.x, p.y, 255, 0, 0, id_prefix + "text");
    }

    void addMarker3d(PCLVisualizer & visualizer, const PointCloud<PointXYZ> & cloud, const string & id_prefix) {
        for (size_t i = 0; i < cloud.size(); i++) {
            PointXYZ p = *(cloud.begin() + i);
            addMarker3d(visualizer, p, str(boost::format("%s-%d") % id_prefix % i));
        }
    }


}

