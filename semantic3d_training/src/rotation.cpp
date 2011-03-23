/**
 * Author: Julius Adorf
 */

#include "rotation.h"

#include <math.h>

using namespace cv;

void populateXRotationMatrix(Mat rot, double angle) {
    rot.at<double>(0, 0) = 1;
    rot.at<double>(0, 1) = 0;
    rot.at<double>(0, 2) = 0;
    rot.at<double>(1, 0) = 0;
    rot.at<double>(1, 1) = cos(angle);
    rot.at<double>(1, 2) = -sin(angle);
    rot.at<double>(2, 0) = 0;
    rot.at<double>(2, 1) = sin(angle);
    rot.at<double>(2, 2) = cos(angle);
}

void populateYRotationMatrix(Mat rot, double angle) {
    rot.at<double>(0, 0) = cos(angle);
    rot.at<double>(0, 1) = 0;
    rot.at<double>(0, 2) = sin(angle);
    rot.at<double>(1, 0) = 0;
    rot.at<double>(1, 1) = 1;
    rot.at<double>(1, 2) = 0;
    rot.at<double>(2, 0) = -sin(angle);
    rot.at<double>(2, 1) = 0;
    rot.at<double>(2, 2) = cos(angle);
}

void populateZRotationMatrix(Mat rot, double angle) {
    rot.at<double>(0, 0) = cos(angle);
    rot.at<double>(0, 1) = -sin(angle);
    rot.at<double>(0, 2) = 0;
    rot.at<double>(1, 0) = sin(angle);
    rot.at<double>(1, 1) = cos(angle);
    rot.at<double>(1, 2) = 0;
    rot.at<double>(2, 0) = 0;
    rot.at<double>(2, 1) = 0;
    rot.at<double>(2, 2) = 1;
}
