/*
 * Author: Julius Adorf
 */

#include "test.h"
#include "viz.h"

#include <tod/core/Features2d.h>
#include <gtest/gtest.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace tod;
using namespace cv;
using namespace clutseg;

void sampleFeatures(Features2d & f2d) {
    FileStorage fs("./data/image_00000.png.features.yaml.gz", FileStorage::READ);
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = imread("./data/image_00000.png", 0);
}

void sampleGuess(Guess & guess, Features2d & f2d) {
    sampleFeatures(f2d);
    for (size_t i = 0; i < f2d.keypoints.size(); i++) {
        guess.image_points_.push_back(f2d.keypoints[i].pt);
    }
    for (size_t i = 0; i < f2d.keypoints.size(); i += 3) {
        guess.inliers.push_back(i);
    }
}

void sampleGuess(Guess & guess) {
    Features2d f2d;
    sampleGuess(guess, f2d);
}

TEST(Viz, DrawKeypoints) {
    // Load a color image
    Mat canvas = imread("./data/image_00000.png");
    EXPECT_EQ(CV_8UC3, canvas.type());
    // Load keypoints 
    Features2d f2d;
    sampleFeatures(f2d);
    // Convert image to grayscale
    f2d.image = Mat(canvas.rows, canvas.cols, CV_8UC1);
    EXPECT_EQ(CV_8UC1, f2d.image.type());
    //drawKeypoints(f2d.image, f2d.keypoints, canvas, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG); 
    clutseg::drawKeypoints(canvas, f2d.keypoints);
    EXPECT_EQ(CV_8UC3, canvas.type());
    imshow("DrawKeypoints", canvas);
    waitKey(-1);
}

TEST(Viz, DrawInliers) {
    Guess guess;
    sampleGuess(guess);
    Mat canvas = imread("./data/image_00000.png");
    drawInliers(canvas, guess); 
    imshow("DrawInliers", canvas);
    waitKey(-1);
}

TEST(Viz, DrawInliersAndKeypoints) {
    Guess guess;
    Features2d f2d;
    sampleGuess(guess, f2d);
    Mat canvas = imread("./data/image_00000.png");
    drawKeypoints(canvas, f2d.keypoints); 
    drawInliers(canvas, guess); 
    imshow("DrawInliersAndKeypoints", canvas);
    waitKey(-1);
}

