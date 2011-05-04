/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/pose_util.h"
#include "clutseg/viz.h"

#include <gtest/gtest.h>
#include <fiducial/fiducial.h>
#include <cv.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv_candidate/Camera.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace opencv_candidate;
using namespace fiducial;
using namespace clutseg;

TEST(PoseUtil, PoseDrawer) {
    PoseRT pose;
    FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    pose.read(in[PoseRT::YAML_NODE_NAME]);
    Mat img = imread("./data/fat_free_milk_image_00000.png");
    Mat canvas = img.clone();
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    PoseDrawer(canvas, camera.K, pose);
    imshow("PoseDrawer", canvas);
    waitKey(0);
}


TEST(PoseUtil, PoseToPoseRT) {
    // PoseRT posert;
    // FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    // posert.read(in[PoseRT::YAML_NODE_NAME]);
    PoseRT posert;
    readPose("./data/fat_free_milk_image_00000.png.pose.yaml", posert);
    
    Pose pose;
    poseRtToPose(posert, pose);
    PoseRT posert2;
    poseToPoseRT(pose, posert2);

    EXPECT_DOUBLE_EQ(posert.tvec.at<double>(0, 0), posert.tvec.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(posert.tvec.at<double>(1, 0), posert.tvec.at<double>(1, 0));
    EXPECT_DOUBLE_EQ(posert.tvec.at<double>(2, 0), posert.tvec.at<double>(2, 0));
    EXPECT_DOUBLE_EQ(posert.rvec.at<double>(0, 0), posert.rvec.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(posert.rvec.at<double>(1, 0), posert.rvec.at<double>(1, 0));
    EXPECT_DOUBLE_EQ(posert.rvec.at<double>(2, 0), posert.rvec.at<double>(2, 0));

    Mat canvas = imread("./data/fat_free_milk_image_00000.png");
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, pose, camera);
    imshow("PoseToPoseRT", canvas);
    waitKey(0);
}

TEST(PoseUtil, WritePoseRT) {
    PoseRT posert;
    writePose("./data/writeposert.yaml", posert);
}

TEST(PoseUtil, WritePose) {
    Pose pose;
    writePose("./data/writepose.yaml", pose);
}

TEST(PoseUtil, TestTranslatePose) {
    PoseRT pose_zero;
    PoseRT pose_m15;
    PoseRT pose_p13;
    readPose("./data/image_00042.png.pose.yaml", pose_zero);
    Mat t_m15 = (Mat_<double>(3, 1) << -0.15, 0, 0);
    Mat t_p13 = (Mat_<double>(3, 1) << 0.13, 0, 0);
    translatePose(pose_zero, t_m15, pose_m15);    
    translatePose(pose_zero, t_p13, pose_p13);    
    // TODO: introduce fixture
    Mat canvas = imread("./data/image_00042.png");
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, pose_m15, camera);
    drawPose(canvas, pose_zero, camera);
    drawPose(canvas, pose_p13, camera);
    imshow("TestTranslatePose", canvas);
    waitKey(0);
}

