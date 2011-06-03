/**
 * Author: Julius Adorf
 */

#include "clutseg/pcl_visualization_addons.h"

#include "clutseg/common.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <opencv_candidate/PoseRT.h>
    #include <pcl_visualization/pcl_visualizer.h>
    #include <cv.h>
    #include <assert.h>
    #include <boost/format.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace pcl;
using namespace pcl_visualization;
using namespace opencv_candidate;
using namespace cv;

// see (pending?)
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

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

    void addMarker3d(PCLVisualizer & visualizer, const PointXYZ & marker, int r, int g, int b, const string & id_prefix) {
        visualizer.addSphere(marker, 0.01, r / 255.0, g / 255.0, b / 255.0, id_prefix + "sphere");
    }

    void addMarker3d(PCLVisualizer & visualizer, const PointCloudT & markers, int r, int g, int b,const string & id_prefix) {
        for (size_t i = 0; i < markers.size(); i++) {
            PointXYZ p = *(markers.begin() + i);
            addMarker3d(visualizer, p, r, g, b, str(boost::format("%s-%d") % id_prefix % i));
        }
    }

    void addMarkerPolygon3d(PCLVisualizer & visualizer,
                            const PointCloudT & markers,
                            int r, int g, int b,
                            const string & id_prefix) {
        addMarker3d(visualizer, markers, r, g, b, id_prefix);
        for (PointCloudT::const_iterator it = markers.begin(),
             end = markers.end(); it != end; it++) {
            string id = str(boost::format("%s-%d-line") % id_prefix % (it - markers.begin()));
            visualizer.addLine(*it, *(it == (end-1) ? markers.begin() : it+1), r, g, b, id);
        }
    }

}

