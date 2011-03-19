/* 
 * Author: Julius Adorf
 */

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
    namedWindow("image");
    imshow("image", img);
    namedWindow("gray-level image");
    imshow("gray-level image", img2);
    waitKey(5000);
}

