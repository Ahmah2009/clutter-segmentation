/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    // TODO: get rid of parameter sameScaleH. Empirical tests (see test_map.cpp) have shown
    // that scaling the indices only using width ratio (even for the y indices) produce better
    // results.

    void mapToCloud(pcl::PointCloud<pcl::PointXYZ> & keypoints3d,
                    const std::vector<cv::Point> keypoints2d,
                    const cv::Mat & scene2d,
                    const pcl::PointCloud<pcl::PointXYZ> & scene3d, bool sameScaleH = true);

    void mapInliersToCloud(pcl::PointCloud<pcl::PointXYZ> & keypoints3d, const tod::Guess & guess,
                    const cv::Mat & scene2d, const pcl::PointCloud<pcl::PointXYZ> & scene3d, bool sameScaleH = true);
}

