/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/pose.h"
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

// TODO: consistently name tests

struct PoseTest : public ::testing::Test {

    void SetUp() {
        readPose("./data/image_00000.png.pose.yaml", posert);
        readPose("./data/image_00042.png.pose.yaml", posert3);

        canvas = imread("./data/image_00000.png");
        canvas3 = imread("./data/image_00042.png");
        camera = Camera("./data/camera.yml", Camera::TOD_YAML);
    }

    PoseRT posert;    
    PoseRT posert3;

    Mat canvas;
    Mat canvas3;
    Camera camera;
};

TEST_F(PoseTest, PoseToPoseRT) {
    // identity operation
    PoseRT posertB = poseToPoseRT(poseRtToPose(posert));

    EXPECT_NEAR(posert.tvec.at<double>(0, 0), posertB.tvec.at<double>(0, 0), 1e-6);
    EXPECT_NEAR(posert.tvec.at<double>(1, 0), posertB.tvec.at<double>(1, 0), 1e-6);
    EXPECT_NEAR(posert.tvec.at<double>(2, 0), posertB.tvec.at<double>(2, 0), 1e-6);
    EXPECT_NEAR(posert.rvec.at<double>(0, 0), posertB.rvec.at<double>(0, 0), 1e-6);
    EXPECT_NEAR(posert.rvec.at<double>(1, 0), posertB.rvec.at<double>(1, 0), 1e-6);
    EXPECT_NEAR(posert.rvec.at<double>(2, 0), posertB.rvec.at<double>(2, 0), 1e-6);

    drawPose(canvas, pose, camera);
    imshow("PoseToPoseRT", canvas);
    waitKey(0);
}

TEST_F(PoseTest, WritePoseRT) {
    writePose("./build/writeposert.yaml", posert);
}

TEST_F(PoseTest, TestTranslatePose) {
    PoseRT pose_zero = posert3;
    PoseRT pose_m15;
    PoseRT pose_p13;
    Mat t_m15 = (Mat_<double>(3, 1) << -0.15, 0, 0);
    Mat t_p13 = (Mat_<double>(3, 1) << 0.13, 0, 0);
    pose_m15 = translatePose(pose_zero, t_m15);    
    pose_p13 = translatePose(pose_zero, t_p13);    
    drawPose(canvas3, pose_m15, camera);
    drawPose(canvas3, pose_zero, camera);
    drawPose(canvas3, pose_p13, camera);
    imshow("TestTranslatePose", canvas3);
    waitKey(0);
}

TEST_F(PoseTest, ValidatePose) {
    float x = posert3.rvec.at<float>(0, 0);
    float y = posert3.rvec.at<float>(1, 0);
    float z = posert3.rvec.at<float>(2, 0);
    EXPECT_GT(0.0f, x);
    EXPECT_GT(0.0f, y);
    EXPECT_LT(0.0f, z);
}

TEST_F(PoseTest, Norm) {
    double x = posert.rvec.at<double>(0, 0);
    double y = posert.rvec.at<double>(1, 0);
    double z = posert.rvec.at<double>(2, 0);
    double l2 = sqrt(x*x+y*y+z*z);
    EXPECT_DOUBLE_EQ(l2, norm(posert.rvec));
}

TEST_F(PoseTest, ArcusCosinus) {
    EXPECT_DOUBLE_EQ(M_PI / 3.0, acos(0.5));
}

TEST_F(PoseTest, ArcusCosinusZero) {
    EXPECT_DOUBLE_EQ(0, acos(1.0));
}

TEST_F(PoseTest, ZeroAngleBetween) {
    EXPECT_DOUBLE_EQ(0.0f, angleBetweenVectors(posert.rvec, posert.rvec));  
}

TEST_F(PoseTest, AngleBetween) {
    PoseRT p = posert3;
    PoseRT q = posert3;
    q.rvec *= 2;
    EXPECT_DOUBLE_EQ(0.0f, angleBetweenVectors(p.rvec, q.rvec));  
}

TEST_F(PoseTest, DiffRotationIdentity) {
    PoseRT p = posert3;
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

TEST_F(PoseTest, DiffTwentyDegrees) {
    PoseRT p = posert3;
    PoseRT q;
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    Mat rn = q.rvec / norm(q.rvec);
    q.rvec += M_PI / 9.0 * rn;

    drawPose(canvas3, p, camera);
    drawPose(canvas3, q, camera);
    imshow("DiffTwentyDegrees", canvas3);
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

TEST_F(PoseTest, DiffTwentyDegrees2) {
    PoseRT p = posert3;
    PoseRT q;
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    Mat r = randomOrientation(M_PI / 9.0);

    Mat R;
    Rodrigues(r, R);
    Mat Q;
    Rodrigues(q.rvec, Q);
    Rodrigues(R*Q, q.rvec);

    drawPose(canvas3, p, camera);
    drawPose(canvas3, q, camera);
    imshow("DiffTwentyDegrees2", canvas3);
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

TEST_F(PoseTest, AngleBetweenOrientationsTwentyDegrees) {
    PoseRT p = posert3;
    PoseRT q;
    
    q.tvec = p.tvec.clone();
    q.rvec = p.rvec.clone();

    Mat r = randomOrientation(M_PI / 9.0);

    Mat R;
    Rodrigues(r, R);
    Mat Q;
    Rodrigues(q.rvec, Q);
    Rodrigues(R*Q, q.rvec);

    EXPECT_NEAR(M_PI / 9.0, angleBetweenOrientations(p, q), 1e-10);
} 

/* Check whether invariant Q = P * diffRotation(P, Q) holds. */
TEST_F(PoseTest, rotatePose) {
    PoseRT p = posert3;

    Mat r = randomOrientation(M_PI / 9.0);

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

