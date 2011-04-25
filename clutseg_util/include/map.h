/*
 * Author: Julius Adorf
 */

#include "clutseg.h"
#include <cv.h>
#include <vector>

namespace clutseg {

    void mapToCloud(PointCloudT & keypoints3d, const vector<Point> keypoints2d,
                    const Mat & scene2d, const PointCloudT & scene3d);

    void mapInliersToCloud(PointCloudT & keypoints3d, const Guess & guess,
                    const Mat & scene2d, const PointCloudT & scene3d);
}

