/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "pose_util.h"
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

TEST(PoseUtil, PoseDrawerFunction) {
    PoseRT pose;
    FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    pose.read(in[PoseRT::YAML_NODE_NAME]);
    Mat img = imread("./data/fat_free_milk_image_00000.png");
    Mat canvas = img.clone();
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, pose, camera);
    imshow("PoseDrawerFunction", canvas);
    waitKey(0);
}

TEST(PoseUtil, PoseToPoseRT) {
    PoseRT posert;
    FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    posert.read(in[PoseRT::YAML_NODE_NAME]);
    
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

    Mat img = imread("./data/fat_free_milk_image_00000.png");
    Mat canvas = img.clone();
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

