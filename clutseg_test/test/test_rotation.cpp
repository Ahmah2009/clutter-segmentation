/**
 * Author: Julius Adorf
 */

#include "test.h"

#include <cv.h>
#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "rotation.h"

using namespace cv;
using namespace clutseg;

TEST(Rotation, RotateAroundXAxis) {
    Mat rx = generateXRotationMatrix(M_PI / 2);
    Mat yaxis = Mat(3, 1, CV_64F);
    yaxis.at<double>(0, 0) = 0;
    yaxis.at<double>(1, 0) = 1;
    yaxis.at<double>(2, 0) = 0;
    Mat zaxis = Mat(3, 1, CV_64F);
    zaxis.at<double>(0, 0) = 0;
    zaxis.at<double>(1, 0) = 0;
    zaxis.at<double>(2, 0) = 1;
    // Now rotate a yaxis unit vector, resulting vector should be
    // aligned with zaxis
    Mat v = rx*yaxis;
    EXPECT_NEAR(v.at<double>(0, 0), zaxis.at<double>(0, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(1, 0), zaxis.at<double>(1, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(2, 0), zaxis.at<double>(2, 0), 0.0001);
}

TEST(Rotation, RotateAroundYAxis) {
    Mat ry = generateYRotationMatrix(M_PI / 2);
    Mat zaxis = Mat(3, 1, CV_64F);
    zaxis.at<double>(0, 0) = 0;
    zaxis.at<double>(1, 0) = 0;
    zaxis.at<double>(2, 0) = 1;
    Mat xaxis = Mat(3, 1, CV_64F);
    xaxis.at<double>(0, 0) = 1;
    xaxis.at<double>(1, 0) = 0;
    xaxis.at<double>(2, 0) = 0;
    // Now rotate a zaxis unit vector, resulting vector should be
    // aligned with xaxis
    Mat v = ry*zaxis;
    EXPECT_NEAR(v.at<double>(0, 0), xaxis.at<double>(0, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(1, 0), xaxis.at<double>(1, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(2, 0), xaxis.at<double>(2, 0), 0.0001);
}

TEST(Rotation, RotateAroundZAxis) {
    Mat rz = generateZRotationMatrix(M_PI / 2);
    Mat xaxis = Mat(3, 1, CV_64F);
    xaxis.at<double>(0, 0) = 1;
    xaxis.at<double>(1, 0) = 0;
    xaxis.at<double>(2, 0) = 0;
    Mat yaxis = Mat(3, 1, CV_64F);
    yaxis.at<double>(0, 0) = 0;
    yaxis.at<double>(1, 0) = 1;
    yaxis.at<double>(2, 0) = 0;
    // Now rotate a xaxis unit vector, resulting vector should be
    // aligned with yaxis
    Mat v = rz*xaxis;
    EXPECT_NEAR(v.at<double>(0, 0), yaxis.at<double>(0, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(1, 0), yaxis.at<double>(1, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(2, 0), yaxis.at<double>(2, 0), 0.0001);
}

TEST(Rotation, RotateAroundYZAxes) {
    Mat ry = generateYRotationMatrix(M_PI / 2);
    Mat rz = generateZRotationMatrix(M_PI / 2);
    Mat yaxis = Mat(3, 1, CV_64F);
    yaxis.at<double>(0, 0) = 0;
    yaxis.at<double>(1, 0) = 1;
    yaxis.at<double>(2, 0) = 0;
    Mat zaxis = Mat(3, 1, CV_64F);
    zaxis.at<double>(0, 0) = 0;
    zaxis.at<double>(1, 0) = 0;
    zaxis.at<double>(2, 0) = 1;
    // Now rotate a zaxis unit vector 90 degrees around y-axis and then 90
    // degrees around z-axis, resulting vector should be aligned with yaxis
    Mat v = rz*ry*zaxis;
    EXPECT_NEAR(v.at<double>(0, 0), yaxis.at<double>(0, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(1, 0), yaxis.at<double>(1, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(2, 0), yaxis.at<double>(2, 0), 0.0001);
}

TEST(Rotation, IdentityRotation) {
    Mat rx = generateXRotationMatrix(M_PI / 2);
    Mat ry = generateYRotationMatrix(M_PI / 2);
    Mat rz = generateZRotationMatrix(M_PI / 2);
    Mat xaxis = Mat(3, 1, CV_64F);
    xaxis.at<double>(0, 0) = 1;
    xaxis.at<double>(1, 0) = 0;
    xaxis.at<double>(2, 0) = 0;
    Mat v = ry*rx*rz*xaxis;
    EXPECT_NEAR(v.at<double>(0, 0), xaxis.at<double>(0, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(1, 0), xaxis.at<double>(1, 0), 0.0001);
    EXPECT_NEAR(v.at<double>(2, 0), xaxis.at<double>(2, 0), 0.0001);
}

TEST(Rotation, Rodrigues2Conversion) {
    Mat rotm = generateXRotationMatrix(M_PI / 2);
    Mat rvec = Mat(3, 1, CV_64F);
    Rodrigues(rotm, rvec);
    Mat rotm2 = Mat(3, 3, CV_64F);
    Rodrigues(rvec, rotm2);
    EXPECT_TRUE(norm(rotm, rotm2) < 0.0001);
}

