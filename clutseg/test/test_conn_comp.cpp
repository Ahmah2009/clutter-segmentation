/*
 * Author: Julius Adorf
 */

#include "clutseg/conn_comp.h"

#include <gtest/gtest.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

struct ConnCompTest : public ::testing::Test {

    void SetUp() {
        img = imread("./data/mask.png", 0);
    }

    Mat img;
};


TEST_F(ConnCompTest, ReadMask) {
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            EXPECT_TRUE(img.at<uint8_t>(i, j) == 0 || img.at<uint8_t>(i, j) == 255);
        }
    }
}

TEST_F(ConnCompTest, FillSmallConnectedComponents) {
    ASSERT_FALSE(img.empty());
    img = img > 1;
    imshow("Source",img);
    largestConnectedComponent(img);
    imshow("Components", img);
    waitKey(0);
}

