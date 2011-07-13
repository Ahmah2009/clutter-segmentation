/*
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

    /** \brief Maps 2D points in a scene to the corresponding 3D points. */
    void mapToCloud(PointCloudT & keypoints3d,
                    const std::vector<cv::Point> keypoints2d,
                    const cv::Mat & scene2d,
                    const PointCloudT & scene3d);

    /** \brief Maps the inliers (here: keypoints) in a scene to the
     * corresponding 3D points. */
    void mapInliersToCloud(PointCloudT & keypoints3d, const tod::Guess & guess,
                    const cv::Mat & scene2d, const PointCloudT & scene3d);
}

#endif
