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

// TODO: rename PoseUtil
// TODO: fixture

struct PoseUtilTest : public ::testing::Test {

};

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
// TODO: refactor

TEST(PoseUtil, ArcusCosinus) {
    EXPECT_DOUBLE_EQ(M_PI / 3.0, acos(0.5));
}

TEST(PoseUtil, ArcusCosinusZero) {
    EXPECT_DOUBLE_EQ(0, acos(1.0));
}

TEST(PoseUtil, ZeroAngleBetween) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);
    EXPECT_DOUBLE_EQ(0.0f, angleBetweenVectors(p.rvec, p.rvec));  
}

TEST(PoseUtil, AngleBetween) {
    PoseRT p;
    PoseRT q;
    readPose("./data/image_00042.png.pose.yaml", p);
    readPose("./data/image_00042.png.pose.yaml", q);
    q.rvec *= 2;
    EXPECT_DOUBLE_EQ(0.0f, angleBetweenVectors(p.rvec, q.rvec));  
}

TEST(PoseUtil, DiffRotationIdentity) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);
    Mat R;
    Rodrigues(p.rvec, R);
    Mat I = diffRotation(R, R);
    EXPECT_NEAR(1.0, I.at<double>(0, 0), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(0, 1), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(0, 2), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(1, 0), 1e-10);
    EXPECT_NEAR(1.0, I.at<double>(1, 1), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(1, 2), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(2, 0), 1e-10);
    EXPECT_NEAR(0.0, I.at<double>(2, 1), 1e-10);
    EXPECT_NEAR(1.0, I.at<double>(2, 2), 1e-10);
}

TEST(PoseUtil, DiffTwentyDegrees) {
    PoseRT p;
    PoseRT q;
    readPose("./data/image_00042.png.pose.yaml", p);
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    Mat rn = q.rvec / norm(q.rvec);
    q.rvec += M_PI / 9.0 * rn;

    Mat canvas = imread("./data/image_00042.png");
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, p, camera);
    drawPose(canvas, q, camera);
    imshow("DiffTwentyDegrees", canvas);
    waitKey(0);

    Mat R1;
    Mat R2;
    Rodrigues(p.rvec, R1);
    Rodrigues(q.rvec, R2);
    Mat D = diffRotation(R1, R2);
    Vec3d r;
    Rodrigues(D, r);
    EXPECT_NEAR(M_PI / 9.0, norm(r), 1e-10);
}

TEST(PoseUtil, DiffTwentyDegrees2) {
    PoseRT p;
    PoseRT q;
    readPose("./data/image_00042.png.pose.yaml", p);
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    // Create some random rotation about twenty degrees.
    Mat r = Mat::zeros(3, 1, CV_64FC1);
    r.at<double>(0, 0) = rand() - 0.5; 
    r.at<double>(1, 0) = rand() - 0.5; 
    r.at<double>(2, 0) = rand() - 0.5; 
    r = (M_PI / 9.0) * (r / norm(r));

    Mat R;
    Rodrigues(r, R);
    Mat Q;
    Rodrigues(q.rvec, Q);
    Rodrigues(R*Q, q.rvec);

    Mat canvas = imread("./data/image_00042.png");
    Camera camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    drawPose(canvas, p, camera);
    drawPose(canvas, q, camera);
    imshow("DiffTwentyDegrees2", canvas);
    waitKey(0);
 
    Mat R1;
    Mat R2;
    Rodrigues(p.rvec, R1);
    Rodrigues(q.rvec, R2);
    Mat D = diffRotation(R1, R2);
    Mat d;
    Rodrigues(D, d);
    EXPECT_NEAR(M_PI / 9.0, norm(d), 1e-10);
}

TEST(PoseUtil, AngleBetweenOrientationsTwentyDegrees) {
    PoseRT p;
    PoseRT q;
    readPose("./data/image_00042.png.pose.yaml", p);
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    // Create some random rotation about twenty degrees.
    Mat r = Mat::zeros(3, 1, CV_64FC1);
    r.at<double>(0, 0) = rand() - 0.5; 
    r.at<double>(1, 0) = rand() - 0.5; 
    r.at<double>(2, 0) = rand() - 0.5; 
    r = (M_PI / 9.0) * (r / norm(r));

    Mat R;
    Rodrigues(r, R);
    Mat Q;
    Rodrigues(q.rvec, Q);
    Rodrigues(R*Q, q.rvec);

    EXPECT_NEAR(M_PI / 9.0, angleBetweenOrientations(p, q), 1e-10);
} 

/* Check whether invariant Q = P * diffRotation(P, Q) holds. */
TEST(PoseUtil, rotatePose) {
    PoseRT p;
    readPose("./data/image_00042.png.pose.yaml", p);

    // Create some random rotation about twenty degrees.
    Mat r = Mat::zeros(3, 1, CV_64FC1);
    r.at<double>(0, 0) = rand() - 0.5; 
    r.at<double>(1, 0) = rand() - 0.5; 
    r.at<double>(2, 0) = rand() - 0.5; 
    r = (M_PI / 9.0) * (r / norm(r));

    PoseRT q = rotatePose(p, r);
    
    Mat P;
    Mat Q;
    Rodrigues(p.rvec, P); 
    Rodrigues(q.rvec, Q); 
    Mat D = diffRotation(P, Q);
    Mat Q_ = P * D;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Q.at<double>(i, j), Q_.at<double>(i, j), 1e-10);
        }
    }

    Mat d = Mat::zeros(3, 1, CV_64FC1);
    Rodrigues(D, d);
    EXPECT_NEAR(M_PI / 9.0, angleBetweenOrientations(p, q), 1e-10);
}

