/**
 * Author: Julius Adorf
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "clutseg/common.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <sqlite3.h>
    #include <tod/detecting/GuessGenerator.h>
    #include <vector>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    // TODO: get rid of parameter sameScaleH. Empirical tests (see test_map.cpp) have shown
    // that scaling the indices only using width ratio (even for the y indices) produce better
    // results.

    void mapToCloud(PointCloudT & keypoints3d,
                    const std::vector<cv::Point> keypoints2d,
                    const cv::Mat & scene2d,
                    const PointCloudT & scene3d, bool sameScaleH = true);

    void mapInliersToCloud(PointCloudT & keypoints3d, const tod::Guess & guess,
                    const cv::Mat & scene2d, const PointCloudT & scene3d, bool sameScaleH = true);
}

#endif
