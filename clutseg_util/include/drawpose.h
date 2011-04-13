/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv_candidate/Camera.h>

using namespace cv;
using namespace opencv_candidate;

/** Draws a given pose. The pose is projected on the canvas using
 * the camera information. The result shows a coordinate system that
 * visualizes the pose on the image. */
void drawPose(const PoseRT & pose, const Mat & image, const Camera & camera, Mat & canvas);

void drawPose(const Pose & pose, const Mat & image, const Camera & camera, Mat & canvas);

