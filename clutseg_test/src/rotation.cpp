/**
 * Author: Julius Adorf
 */

#include "rotation.h"

#include <math.h>

namespace clutseg {

    Mat generateXRotationMatrix(double angle) {
        Mat rot = Mat(3, 3, CV_64F);
        rot.at<double>(0, 0) = 1;
        rot.at<double>(0, 1) = 0;
        rot.at<double>(0, 2) = 0;
        rot.at<double>(1, 0) = 0;
        rot.at<double>(1, 1) = cos(angle);
        rot.at<double>(1, 2) = -sin(angle);
        rot.at<double>(2, 0) = 0;
        rot.at<double>(2, 1) = sin(angle);
        rot.at<double>(2, 2) = cos(angle);
        return rot;
    }

    Mat generateYRotationMatrix(double angle) {
        Mat rot = Mat(3, 3, CV_64F);
        rot.at<double>(0, 0) = cos(angle);
        rot.at<double>(0, 1) = 0;
        rot.at<double>(0, 2) = sin(angle);
        rot.at<double>(1, 0) = 0;
        rot.at<double>(1, 1) = 1;
        rot.at<double>(1, 2) = 0;
        rot.at<double>(2, 0) = -sin(angle);
        rot.at<double>(2, 1) = 0;
        rot.at<double>(2, 2) = cos(angle);
        return rot;
    }

    Mat generateZRotationMatrix(double angle) {
        Mat rot = Mat(3, 3, CV_64F);
        rot.at<double>(0, 0) = cos(angle);
        rot.at<double>(0, 1) = -sin(angle);
        rot.at<double>(0, 2) = 0;
        rot.at<double>(1, 0) = sin(angle);
        rot.at<double>(1, 1) = cos(angle);
        rot.at<double>(1, 2) = 0;
        rot.at<double>(2, 0) = 0;
        rot.at<double>(2, 1) = 0;
        rot.at<double>(2, 2) = 1;
        return rot;
    }

}

