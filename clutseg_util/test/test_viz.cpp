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

void sampleColorImage(Mat & img) {
    img = imread("./data/image_00000.png");
}

void sampleGuess(Guess & guess, Features2d & f2d) {
    sampleFeatures(f2d);
    vector<int> selection(f2d.keypoints.size());
    for (size_t i = 0; i < f2d.keypoints.size(); i++) {
        guess.image_points_.push_back(f2d.keypoints[i].pt);
        selection.push_back(i);
    }
    random_shuffle(selection.begin(), selection.end()); 
    for (size_t i = 0; i < f2d.keypoints.size() / 3; i ++) {
        guess.inliers.push_back(selection[i]);
    }
}

void sampleGuess(Guess & guess) {
    Features2d f2d;
    sampleGuess(guess, f2d);
}

void samplePose(PoseRT & pose) {
    FileStorage in("./data/image_00000.png.pose.yaml", FileStorage::READ);
    pose.read(in[PoseRT::YAML_NODE_NAME]);
}

void sampleCamera(Camera & camera) {
    camera = Camera("./data/camera.yml", Camera::TOD_YAML);
}

class Viz : public ::testing::Test {
    public:
        virtual void SetUp() {
            sampleCamera(camera);
            samplePose(pose);
            sampleGuess(guess1);
            sampleGuess(guess2);
            sampleFeatures(f2d);
            sampleColorImage(colorImage);
        }

        Camera camera;
        PoseRT pose;
        Guess guess1;
        Guess guess2;
        Features2d f2d;
        Mat colorImage;
};

TEST_F(Viz, DrawKeypoints) {
    Mat canvas = colorImage;
    EXPECT_EQ(CV_8UC3, canvas.type());
    EXPECT_EQ(CV_8UC1, f2d.image.type());
    clutseg::drawKeypoints(canvas, f2d.keypoints);
    EXPECT_EQ(CV_8UC3, canvas.type());
    imshow("DrawKeypoints", canvas);
    waitKey(-1);
}

TEST_F(Viz, DrawInliers) {
    Mat canvas = colorImage; 
    drawInliers(canvas, guess1); 
    imshow("DrawInliers", canvas);
    waitKey(-1);
}

TEST_F(Viz, DrawMoreInliers) {
    Mat canvas = colorImage; 
    drawInliers(canvas, guess1); 
    drawInliers(canvas, guess2, Scalar(255, 30, 0)); 
    imshow("DrawMoreInliers", canvas);
    waitKey(-1);
}

TEST_F(Viz, DrawInliersAndKeypoints) {
    Mat canvas = colorImage; 
    drawKeypoints(canvas, f2d.keypoints); 
    drawInliers(canvas, guess1); 
    imshow("DrawInliersAndKeypoints", canvas);
    waitKey(-1);
}

TEST_F(Viz, DrawPose) {
    Mat canvas = colorImage; 
    drawPose(canvas, pose, camera);
    imshow("DrawPose", canvas);
    waitKey(0);
}

TEST_F(Viz, DrawPoseOnWhiteCanvas) {
    Mat canvas = Mat::zeros(1024, 1280, CV_8UC3);
    canvas = Scalar(255, 255, 255);
    drawPose(canvas, pose, camera);
    imshow("DrawPoseOnWhiteCanvas", canvas);
    waitKey(0);
}

TEST_F(Viz, DrawPoseInliersKeypoints) {
    Mat canvas = Mat::zeros(1024, 1280, CV_8UC3);
    canvas = Scalar(255, 255, 255);
    drawKeypoints(canvas, f2d.keypoints);
    drawInliers(canvas, guess1);
    drawPose(canvas, pose, camera);
    imshow("DrawPoseInliersKeypoints", canvas);
    waitKey(0);
}

