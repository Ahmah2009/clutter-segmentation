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
using namespace clutseg;

TEST(PoseUtil, PoseToPoseRT) {
    // PoseRT posert;
    // FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    // posert.read(in[PoseRT::YAML_NODE_NAME]);
    PoseRT posert;
    readPose("./data/image_00000.png.pose.yaml", posert);
    
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

    Mat canvas = imread("./data/image_00000.png");
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, pose, camera);
    imshow("PoseToPoseRT", canvas);
    waitKey(0);
}

TEST(PoseUtil, WritePoseRT) {
    PoseRT posert;
    writePose("./build/writeposert.yaml", posert);
}

TEST(PoseUtil, WritePose) {
    Pose pose;
    writePose("./build/writepose.yaml", pose);
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

TEST(PoseUtil, ValidatePose) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);
    float x = p.rvec.at<float>(0, 0);
    float y = p.rvec.at<float>(1, 0);
    float z = p.rvec.at<float>(2, 0);
    EXPECT_GT(0.0f, x);
    EXPECT_GT(0.0f, y);
    EXPECT_LT(0.0f, z);
}

TEST(PoseUtil, Norm) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);
    double x = p.rvec.at<double>(0, 0);
    double y = p.rvec.at<double>(1, 0);
    double z = p.rvec.at<double>(2, 0);
    double l2 = sqrt(x*x+y*y+z*z);
    EXPECT_DOUBLE_EQ(l2, norm(p.rvec));
}

// TODO; Fixture

TEST(PoseUtil, ArcusCosinus) {
    EXPECT_DOUBLE_EQ(M_PI / 3.0, acos(0.5));
}

TEST(PoseUtil, ArcusCosinusZero) {
    EXPECT_DOUBLE_EQ(0, acos(1.0));
}

TEST(PoseUtil, ZeroAngleBetween) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);
    EXPECT_DOUBLE_EQ(0.0f, angleBetween(p.rvec, p.rvec));  
}

TEST(PoseUtil, AngleBetween) {
    PoseRT p;
    PoseRT q;
    readPose("./data/image_00042.png.pose.yaml", p);
    readPose("./data/image_00042.png.pose.yaml", q);
    q.rvec *= 2;
    EXPECT_DOUBLE_EQ(0.0f, angleBetween(p.rvec, q.rvec));  
}

