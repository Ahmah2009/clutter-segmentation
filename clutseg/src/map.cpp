/*
 * Author: Julius Adorf
 */

#include "clutseg/map.h"

#include "clutseg/gcc_diagnostic_disable.h"
#include <cv.h>
#include <boost/foreach.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace std;
using namespace pcl;

namespace clutseg {

    void mapToCloud(PointCloudT & keypoints3d, const vector<Point> keypoints2d,
                    const Mat & scene2d, const PointCloudT & scene3d) {
        // See also tod::PCLToPoints, line 117 of clouds.h in tod:training. I wanted a method
        // that can be tested with different parameters, since I am not sure how to interprete
        // cloud.width and cloud.height. Tests (see test_map.cpp) show that scaling with the
        // width ratio is necessary and the width ratio should better also be used as a scaling
        // factor for y-indices.
        float scaleW = float(scene3d.width) / scene2d.cols;
        float scaleH = scaleW;
        BOOST_FOREACH(Point p, keypoints2d) {
            size_t u = size_t(p.x * scaleW);
            size_t v = size_t(p.y * scaleH);
            if (u < scene3d.width && v < scene3d.height) {
                keypoints3d.push_back(scene3d(u, v));
            } else {
                keypoints3d.push_back(PointXYZ(NAN, NAN, NAN));
            }
        }
    }

    void mapInliersToCloud(PointCloudT & keypoints3d, const tod::Guess & guess,
                    const Mat & scene2d, const PointCloudT & scene3d) {
        vector<Point> inliers;
        BOOST_FOREACH(int idx, guess.inliers) {
            inliers.push_back(guess.image_points_[idx]);
        }
        mapToCloud(keypoints3d, inliers, scene2d, scene3d);
    }   

}

