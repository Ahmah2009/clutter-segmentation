/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    void mapToCloud(pcl::PointCloud<pcl::PointXYZ> & keypoints3d,
                    const std::vector<cv::Point> keypoints2d,
                    const cv::Mat & scene2d,
                    const pcl::PointCloud<pcl::PointXYZ> & scene3d);

    void mapInliersToCloud(pcl::PointCloud<pcl::PointXYZ> & keypoints3d, const tod::Guess & guess,
                    const cv::Mat & scene2d, const pcl::PointCloud<pcl::PointXYZ> & scene3d);
}

