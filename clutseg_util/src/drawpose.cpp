/*
 * Author: Julius Adorf
 */

#include <drawpose.h>
#include <fiducial/fiducial.h>
#include <opencv/cxeigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace fiducial;
using namespace cv;
using namespace opencv_candidate;

void drawPose(const PoseRT & pose, const Mat & image, const Camera & camera, Mat & canvas) {
    PoseDrawer(canvas, camera.K, pose);
}

void drawPose(const Pose & pose, const Mat & image, const Camera & camera, Mat & canvas) {
    PoseRT posert;
    Mat R;
    eigen2cv(pose.t(), posert.tvec);
    eigen2cv(pose.r(), R);
    Rodrigues(R, posert.rvec);
    drawPose(posert, image, camera, canvas);
}

