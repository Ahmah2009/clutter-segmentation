
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace cv;

// see http://opencv-users.1802565.n2.nabble.com/Problems-about-the-OpenCV-SIFT-feature-detector-td6084481.html
// see https://code.ros.org/trac/opencv/ticket/1029
// see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html?highlight=siftfeaturedetector#SiftFeatureDetector
int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << "Usage: siftmask <image> <mask>" << endl;
        return 1;
    }

    Mat image = imread(argv[1], 0);
    Mat mask = imread(argv[2], 0);

    assert(CV_8U == mask.type());
    vector<KeyPoint> keypoints;
    SiftFeatureDetector fd = SiftFeatureDetector(SIFT::DetectorParams::
                                         GET_DEFAULT_THRESHOLD(),
                                         SIFT::DetectorParams::
                                         GET_DEFAULT_EDGE_THRESHOLD());
    fd.detect(image, keypoints, mask);
    Mat outImg;
    drawKeypoints(image, keypoints, outImg);
    imshow("mask", mask);
    waitKey(-1);
    imshow("keypoints", outImg);
    waitKey(-1);
    return 0;
}

