/*
 * Author: Julius Adorf
 */

#include "test.h"
#include "clutseg/viz.h"
#include "clutseg/pose.h"

#include <tod/core/Features2d.h>
#include <gtest/gtest.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace tod;
using namespace cv;
using namespace opencv_candidate;
using namespace clutseg;
using namespace std;

void sampleFeatures(Features2d & f2d) {
    FileStorage fs("./data/image_00000.png.features.yaml.gz", FileStorage::READ);
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = imread("./data/image_00000.png", 0);
}

void sampleGuess(Guess & guess, Features2d & f2d) {
    sampleFeatures(f2d);
    vector<int> selection(f2d.keypoints.size());
    for (size_t i = 0; i < f2d.keypoints.size(); i++) {
        guess.image_points_.push_back(f2d.keypoints[i].pt);
        selection[i] = i; // shuffle and truncate later
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


void sampleCamera(Camera & camera) {
    camera = Camera("./data/camera.yml", Camera::TOD_YAML);
}

void sampleText(vector<string> & text) {
    text.push_back("this is the first line");
    text.push_back("this is the second line");
    text.push_back("and the third line");
}


class test_viz : public ::testing::Test {
    public:
        virtual void SetUp() {
            sampleCamera(camera);
            samplePose(pose1);
            samplePose(pose2);
            sampleFeatures(f2d);
            sampleColorImage(colorImage);
            sampleText(text);
            randomizePose(pose2, 0.1, 0.1);
            object = new TexturedObject();
            object->name = "haltbare_milch";
            Pose p1 = poseRtToPose(pose1);
            Pose p2 = poseRtToPose(pose2);
            guess1 = Guess(object, p1, camera.K, camera.D, f2d.image);
            guess2 = Guess(object, p2, camera.K, camera.D, f2d.image);
            sampleGuess(guess1);
            sampleGuess(guess2);
        }

        Camera camera;
        PoseRT pose1;
        PoseRT pose2;
        cv::Ptr<TexturedObject> object;
        Guess guess1;
        Guess guess2;
        Features2d f2d;
        Mat colorImage;
        vector<string> text;
};

TEST_F(test_viz, DrawKeypoints) {
    EXPECT_EQ(CV_8UC3, colorImage.type());
    EXPECT_EQ(CV_8UC1, f2d.image.type());
    clutseg::drawKeypoints(colorImage, f2d.keypoints);
    EXPECT_EQ(CV_8UC3, colorImage.type());
    imshow_and_wait("DrawKeypoints", colorImage);
}

TEST_F(test_viz, DrawInliers) {
    drawInliers(colorImage, guess1); 
    imshow_and_wait("DrawInliers", colorImage);
}

TEST_F(test_viz, DrawInliersBlue) {
    drawInliers(colorImage, guess1, Scalar(255, 0, 0)); 
    imshow_and_wait("DrawInliersBlue", colorImage);
}

TEST_F(test_viz, DrawInliersRandomColor) {
    Scalar color = Scalar(rand() % 256, rand() % 256, rand() % 256);
    drawInliers(colorImage, guess1, color); 
    imshow_and_wait("DrawInliersRandomColor", colorImage);
}

TEST_F(test_viz, DrawMoreInliers) {
    drawInliers(colorImage, guess1); 
    drawInliers(colorImage, guess2, Scalar(255, 30, 0)); 
    imshow_and_wait("DrawMoreInliers", colorImage);
}

TEST_F(test_viz, DrawInliersAndKeypoints) {
    drawKeypoints(colorImage, f2d.keypoints); 
    drawInliers(colorImage, guess1); 
    imshow_and_wait("DrawInliersAndKeypoints", colorImage);
}

TEST_F(test_viz, DrawPose) {
    drawPose(colorImage, pose1, camera);
    imshow_and_wait("DrawPose", colorImage);
}

TEST_F(test_viz, DrawPoseOnWhiteCanvas) {
    Mat canvas = Mat::zeros(1024, 1280, CV_8UC3);
    canvas = Scalar(255, 255, 255);
    drawPose(canvas, pose1, camera);
    imshow_and_wait("DrawPoseOnWhiteCanvas", canvas);
}

TEST_F(test_viz, DrawPoseInliersKeypoints) {
    Mat canvas = Mat::zeros(1024, 1280, CV_8UC3);
    canvas = Scalar(255, 255, 255);
    drawKeypoints(canvas, f2d.keypoints);
    drawInliers(canvas, guess1);
    drawPose(canvas, pose1, camera);
    imshow_and_wait("DrawPoseInliersKeypoints", canvas);
}

TEST_F(test_viz, DrawText) {
    Mat canvas = Mat::zeros(300, 300, CV_8UC3);
    Rect rect = drawText(canvas, text, Point(0, 0), FONT_HERSHEY_PLAIN, 1.0, 2, Scalar::all(255));
    rectangle(canvas, rect.tl(), rect.br(), Scalar::all(204));
    imshow_and_wait("DrawText", canvas);
}

TEST_F(test_viz, DrawGuesses) {
    vector<Guess> guesses;
    vector<PoseRT> poses;
    guesses.push_back(guess1);
    guesses.push_back(guess2);
    poses.push_back(pose1);
    drawGuesses(colorImage, guesses, camera, poses);
    imshow_and_wait("DrawGuesses", colorImage);
}

TEST_F(test_viz, DrawManyGuesses) {
    // check whether we're running out of colors
    vector<Guess> guesses;
    vector<PoseRT> poses;
    for (int i = 0; i < 30; i++) {
        guesses.push_back(guess1);
        guesses.push_back(guess2);
    }
    poses.push_back(pose1);
    drawGuesses(colorImage, guesses, camera, poses);
    imshow_and_wait("DrawManyGuesses", colorImage);
}

