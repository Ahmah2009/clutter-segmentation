/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/conn_comp.h"

#include <gtest/gtest.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

struct test_conn_comp : public ::testing::Test {

    void SetUp() {
        img = imread("./data/mask.png", 0);
    }

    Mat img;
};

TEST_F(test_conn_comp, read_mask) {
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            EXPECT_TRUE(img.at<uint8_t>(i, j) == 0 || img.at<uint8_t>(i, j) == 255);
        }
    }
}

TEST_F(test_conn_comp, largest_connected_component) {
    ASSERT_FALSE(img.empty());
    img = img > 1;
    imshow("Source",img);
    largestConnectedComponent(img);

    if (!fast()) {
        imshow("Components", img);
        waitKey(0);
    }
}
