/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>

using namespace pcl;
using namespace cv;

namespace clutseg {

    void mapToCloud(pcl::PointCloud<PointXYZ> & keypoints3d, const vector<Point> keypoints2d,
                    const Mat & scene2d, const PointCloud<PointXYZ> & scene3d);

    void mapInliersToCloud(PointCloud<PointXYZ> & keypoints3d, const tod::Guess & guess,
                    const Mat & scene2d, const PointCloud<PointXYZ> & scene3d);
}

