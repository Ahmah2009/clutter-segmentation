/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"

#include <gtest/gtest.h>
#include <tod/detecting/Loader.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <stdlib.h>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

// TODO: Create fixture

TEST(ClutsegTest, ChangeOptionsOnline) {
    ClutSegmenter segmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    EXPECT_TRUE(segmenter.getLocateParams().matcherParams.doRatioTest);
    segmenter.getLocateParams().matcherParams.doRatioTest = false;
    EXPECT_FALSE(segmenter.getLocateParams().matcherParams.doRatioTest);
}

TEST(ClutsegTest, DetectionWorks) {
   // Create segmenter
    ClutSegmenter segmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    segmenter.getLocateParams().matcherParams.doRatioTest = false;
    segmenter.getLocateParams().guessParams.ransacIterationsCount = 500;

    Mat queryImage = imread("./data/image_00000.png");
    PointCloudT queryCloud;
    pcl::io::loadPCDFile("./data/cloud_00000.pcd", queryCloud);

    tod::Guess guess;
    PointCloudT inlierCloud;
    bool positive = segmenter.recognize(queryImage, queryCloud, guess, inlierCloud);
    EXPECT_TRUE(positive);
    EXPECT_EQ("haltbare_milch", guess.getObject()->name);
    cout << "detected: " << guess.getObject()->name << endl;
    cout << "inliers:  " << guess.inliers.size() << endl;
}

TEST(ClutsegTest, LoadSingleTrainingBase) {
    vector<Ptr<TexturedObject> > objects;
    Loader loader(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train");
    loader.readTexturedObjects(objects);
    objects.erase(objects.begin() + 1, objects.end());
    EXPECT_EQ(1, objects.size());
    TrainingBase base(objects);
}

TEST(ClutsegTest, SingleDetectionWorks) {
    // Load training base 
    vector<Ptr<TexturedObject> > objects;
    Loader loader(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train");
    loader.readTexturedObjects(objects);
    objects.erase(objects.begin());
    objects.erase(objects.begin() + 2);
    objects.erase(objects.begin() + 3);
    EXPECT_EQ(1, objects.size());
    EXPECT_EQ("haltbare_milch", objects[0]->name);
    TrainingBase base(objects);

    // Load options
    Options opts_;
    FileStorage fs(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml", FileStorage::READ);
    if (!fs.isOpened()) {
        throw ios_base::failure("Cannot read configuration file '" + opts_.config + "'");
    }
    opts_.params.read(fs[tod::TODParameters::YAML_NODE_NAME]);
    // Turning off ratio testing is a very good idea when the training base
    // contains only one object.
    opts_.params.matcherParams.doRatioTest = false;
    opts_.params.guessParams.maxProjectionError = 12.0;
    opts_.params.guessParams.ransacIterationsCount = 1000;
    fs.release();

    // Load query
    Mat queryImage = imread(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/image_00000.png");
    PointCloudT queryCloud;
    pcl::io::loadPCDFile(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/cloud_00000.pcd", queryCloud);

    tod::Guess guess;
    PointCloudT inlierCloud;
 
    Ptr<tod::FeatureExtractor> extractor = tod::FeatureExtractor::create(opts_.params.feParams);
    Ptr<tod::Matcher> rtMatcher = tod::Matcher::create(opts_.params.matcherParams);
    rtMatcher->add(base);
    cv::Ptr<tod::Recognizer> recognizer = new tod::KinectRecognizer(&base, rtMatcher,
                            &opts_.params.guessParams, 0 /* verbose */, opts_.baseDirectory);

    tod::Features2d test;
    test.image = queryImage;
    vector<tod::Guess> guesses;
    extractor->detectAndExtract(test);
    recognizer->match(test, guesses);

    EXPECT_FALSE(guesses.empty());
    guess = guesses[0];
    EXPECT_EQ("haltbare_milch", guess.getObject()->name);
    cout << "detected: " << guess.getObject()->name << endl;
    cout << "inliers:  " << guess.inliers.size() << endl;
}


TEST(ClutsegTest, ias_kinect_1) {
    int c = system("test-eval ias_kinect_train ias_kinect_train/config.yaml ias_kinect_test_train_20");
    EXPECT_EQ(0, c);
}

