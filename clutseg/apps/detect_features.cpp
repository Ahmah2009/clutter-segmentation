/**
 * Author: Julius Adorf
 *
 * Detects keypoints on an image using OpenCV, and writes
 * an image with the keypoints to disk.
 */

#include "clutseg/viz.h"
#include <cv.h>
#include <cstdio>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
   if (argc != 4 && argc != 5) {
        cerr << "Usage: detect_features <type> <src> [<tpl>] <dst>" << endl;
        cerr << endl;
        cerr <<
            "Generates keypoints and writes a visualization of the detected keypoints\n"
            "to disk.\n\n"
            "type - SIFT, SURF, ORB, FAST, STAR, MSER, or any other detector accepted\n"
            "       by OpenCV cv::FeatureDetector::create.\n"
            "src  - The image for which keypoints are to be computed.\n"
            "tpl  - Image on which keypoints are drawn (optional)\n"
            "dst  - Output image\n";
        return -1;
   }

   Ptr<FeatureDetector> detector = FeatureDetector::create(argv[1]);
   Mat img = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
   assert(!img.empty());
   vector<KeyPoint> kpts;
   detector->detect(img, kpts);
   Mat tpl;
   Mat out = Mat::zeros(img.size(), CV_8UC3);
   if (argc == 5) {
        tpl = imread(argv[3]);
   } else {
        tpl = img;
   }
   cv::drawKeypoints(img, kpts, out);
   int idx = (argc == 4 ? 3 : 4);
   imwrite(argv[idx], out);
   return 0;
}
