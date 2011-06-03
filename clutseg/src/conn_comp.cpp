/*
 * Author: Julius Adorf
 */

#include "clutseg/conn_comp.h"

#include "clutseg/gcc_diagnostic_disable.h"
#include <cv.h>
#include <stdint.h>
#include <iostream>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace std;

void largestConnectedComponent(Mat& img) {
    Mat tmp = img.clone(); 
    // Find contours of foreground objects. If none are found, this operation
    // becomes a no-op.
    vector<vector<Point> > contours;
    findContours(tmp, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return;
    }
    
    // Select the largest connected component by counting pixels in every
    // component.
    int max_i = 0;
    int max_n = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        int a = contourArea(Mat(contours[i]));
        if (a > max_n) {
            max_i = i;
            max_n = a;
        }
    }

    // Draw largest connected component on destination image.
    img = Scalar::all(0);
    drawContours(img, contours, max_i, Scalar::all(255), CV_FILLED, 8);
}

