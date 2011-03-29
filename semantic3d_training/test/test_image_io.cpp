/* 
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

using namespace cv;

/** Read and write an image to an image file using OpenCV */
TEST(ImageIO, ReadWriteImage) {
    using namespace boost::filesystem;
    Mat img(imread("./data/icetea2_0000_L.png.cropped.png"));
    imwrite("./data/icetea2_0000_L.png.cropped.out.png", img);
    remove("./data/icetea2_0000_L.png.cropped.out.png");
}

/** Display images using OpenCV */
TEST(ImageIO, DisplayImage) {
    Mat img(imread("./data/icetea2_0000_L.png.cropped.png"));
    Mat img2(imread("./data/icetea2_0000_L.png.cropped.png", 0));
    if (enableUI) {
        // TODO: simplify
        namedWindow("image");
        imshow("image", img);
        namedWindow("gray-level image");
        imshow("gray-level image", img2);
        waitKey(5000);
    }
}

TEST(ImageIO, CreateImage) {
    typedef unsigned char uint8;
    const int r = 410, c = 410;
    Mat img = Mat::zeros(r, c, CV_8U);
    // Create one of those funny optical illusions 
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            if ((i / 10) % 5 == 0 || (j / 10) % 5 == 0) {
                img.at<uint8>(i, j) = 255u;
            }
        }
    }
    if (enableUI) {
        // TODO: simplify
        namedWindow("TEST(ImageIO, CreateImage)");
        imshow("TEST(ImageIO, CreateImage)", img);
        waitKey(5000);
    }
}

