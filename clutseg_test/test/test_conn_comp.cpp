/*
 * Author: Julius Adorf
 */
// TODO: migrate to clutseg_util

#include "test.h"

#include <clutseg/conn_comp.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <stdint.h>
#include <boost/format.hpp>

using namespace cv;
using namespace std;

TEST(ConnComp, ReadMask) {
    Mat image = imread("./data/mask.png", 0);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            EXPECT_TRUE(image.at<uint8_t>(i, j) == 0 || image.at<uint8_t>(i, j) == 255);
        }
    }
}

TEST(ConnComp, FillSmallConnectedComponents) {
    Mat img = imread("data/mask.png", 0);

    img = img > 1;
    namedWindow( "Source", 1 );
    imshow( "Source", img);
    
    largestConnectedComponent(img);

    namedWindow( "Components", 1 );
    imshow( "Components", img);
    waitKey(0);

}

