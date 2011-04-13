/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <fiducial/fiducial.h>
#include <cv.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv_candidate/Camera.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace opencv_candidate;
using namespace fiducial;

TEST(PoseDrawer, PoseDrawer) {
    PoseRT pose;
    FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    pose.read(in[PoseRT::YAML_NODE_NAME]);
    Mat img = imread("./data/fat_free_milk_image_00000.png");
    Mat canvas = img.clone();
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    PoseDrawer(canvas, camera.K, pose);
    imshow("pose", canvas);
    waitKey(0);
}

