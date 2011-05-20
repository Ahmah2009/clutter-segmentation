/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"
#include "clutseg/db.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include <gtest/gtest.h>
#include <limits.h>
#include <tod/detecting/Loader.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

class ClutsegTest : public ::testing::Test {

    public:

        virtual void SetUp() {
            if (!loaded) {
                 sgm = ClutSegmenter(
                    // FIXME: This is a mistake. We cannot use
                    // ias_kinect_test_grounded and ias_kinect_train together.
                    // The ground poses will not match due to different model
                    // coordinate systems. Recognition and estimation of poses
                    // should still work, but comparison to ground truth
                    // becomes invalid.  Also, we must verify that the feature
                    // configurations for training and querying roughly match! 
                    string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
                    string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
                    string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
                );
                loaded = true;
            }

            sgm.setDoLocate(true);
            sgm.resetStats();               
            sgm.getLocateParams().matcherParams.doRatioTest = false;

            haltbare_milch_train_img = imread("./data/image_00000.png");
            pcl::io::loadPCDFile("./data/cloud_00000.pcd", haltbare_milch_train_cloud);

            clutter_img = imread(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/image_00000.png");
            pcl::io::loadPCDFile(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/cloud_00000.pcd", clutter_cloud);
        }

        static ClutSegmenter sgm;
        static bool loaded;
        Mat haltbare_milch_train_img;
        PointCloudT haltbare_milch_train_cloud;
        Mat clutter_img;
        PointCloudT clutter_cloud;

};

ClutSegmenter ClutsegTest::sgm;
bool ClutsegTest::loaded;

/** Verify that changes to parameters do not affect existing ClutSegmenter
 * instances, i.e. that parameters are properly copied in constructor. */
TEST_F(ClutsegTest, ConstructorOpaque) {
    TODParameters detect_params;
    TODParameters locate_params;

    ClutSegmenter s(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        detect_params,
        locate_params
    );
    
    EXPECT_NE("SIFT", detect_params.feParams.detector_type);
    detect_params.feParams.detector_type = "SIFT";
    EXPECT_NE("SIFT", s.getDetectParams().feParams.detector_type);
}

/** Check whether the params returned is actually a reference to the
 * ClutSegmenter's parameters and that changes shine through. In this way,
 * parameter configuration can be changed easily without having to reload the
 * training base. */
TEST_F(ClutsegTest, ChangeParamsOnline) {
    EXPECT_FALSE(sgm.getLocateParams().matcherParams.doRatioTest);
    TODParameters & params = sgm.getLocateParams();
    params.matcherParams.doRatioTest = true;
    EXPECT_TRUE(sgm.getLocateParams().matcherParams.doRatioTest);
}

/** Check whether detection works for a training image. This is expected to
 * return with a whole bunch of inliers since it is only a training image. */
TEST_F(ClutsegTest, RecognizeHaltbareMilch) {
    Guess guess;
    PointCloudT inlierCloud;
    bool positive = sgm.recognize(haltbare_milch_train_img,
                                        haltbare_milch_train_cloud,
                                        guess, inlierCloud);
    EXPECT_TRUE(positive);
    EXPECT_EQ("haltbare_milch", guess.getObject()->name);
    cout << "detected: " << guess.getObject()->name << endl;
    cout << "inliers:  " << guess.inliers.size() << endl;
    EXPECT_GT(guess.inliers.size(), 500);
}

/** Check whether loading a single training base works without failing */
TEST_F(ClutsegTest, LoadSingleTrainingBase) {
    vector<Ptr<TexturedObject> > objects;
    Loader loader(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train");
    loader.readTexturedObjects(objects);
    objects.erase(objects.begin() + 1, objects.end());
    EXPECT_EQ(1, objects.size());
    TrainingBase base(objects);
}

/** Check whether an object is at least detected in clutter */
TEST_F(ClutsegTest, RecognizeInClutter) {
    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 15.0;
    detect_params.guessParams.ransacIterationsCount = 100;
    TODParameters & locate_params = sgm.getLocateParams();
    locate_params.guessParams.maxProjectionError = 8;
    locate_params.guessParams.ransacIterationsCount = 500;
    locate_params.guessParams.minInliersCount = 50;
    //sgm.setAcceptThreshold(20);

    PoseRT ground_pose;
    readPose(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/image_00000.png.pose.yaml", ground_pose);
    PoseRT ground_pose_2 = translatePose(ground_pose, (Mat_<double>(3, 1) << -0.15, 0.0, 0.0));
    PoseRT ground_pose_3 = translatePose(ground_pose, (Mat_<double>(3, 1) << +0.13, 0.0, 0.0));

    // TODO: read from file
    GroundTruth groundTruth;
    groundTruth.labels.push_back(LabeledPose("assam_tea", ground_pose_2));
    groundTruth.labels.push_back(LabeledPose("haltbare_milch", ground_pose));
    groundTruth.labels.push_back(LabeledPose("jacobs_coffee", ground_pose_3));
 
    Guess guess;
    PointCloudT inlierCloud;
    EXPECT_EQ(0, sgm.getStats().choices);
    EXPECT_EQ(0, sgm.getStats().keypoints);
    EXPECT_EQ(0, sgm.getStats().detect_matches);
    EXPECT_EQ(0, sgm.getStats().detect_guesses);
    EXPECT_EQ(0, sgm.getStats().detect_inliers);
    EXPECT_EQ(0, sgm.getStats().detect_choice_matches);
    EXPECT_EQ(0, sgm.getStats().detect_choice_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_matches);
    EXPECT_EQ(0, sgm.getStats().locate_guesses);
    EXPECT_EQ(0, sgm.getStats().locate_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_choice_matches);
    EXPECT_EQ(0, sgm.getStats().locate_choice_inliers);
    bool positive = sgm.recognize(clutter_img, clutter_cloud, guess, inlierCloud);
    ASSERT_TRUE(positive);
    EXPECT_EQ(1, sgm.getStats().choices);
    EXPECT_LT(100, sgm.getStats().keypoints);
    EXPECT_LT(0, sgm.getStats().detect_matches);
    EXPECT_LT(0, sgm.getStats().detect_guesses);
    EXPECT_LT(0, sgm.getStats().detect_inliers);
    EXPECT_LT(0, sgm.getStats().detect_choice_matches);
    EXPECT_LT(0, sgm.getStats().detect_choice_inliers);
    EXPECT_LT(0, sgm.getStats().locate_matches);
    EXPECT_LT(0, sgm.getStats().locate_guesses);
    EXPECT_LT(0, sgm.getStats().locate_inliers);
    EXPECT_LT(0, sgm.getStats().locate_choice_matches);
    EXPECT_EQ(guess.inliers.size(), sgm.getStats().locate_choice_inliers);
    // TODO: get rid of next assertion
    EXPECT_TRUE(guess.getObject()->name == "assam_tea" ||
                guess.getObject()->name == "haltbare_milch" ||
                guess.getObject()->name == "jacobs_coffee");
    EXPECT_TRUE(groundTruth.onScene(guess.getObject()->name));
    EXPECT_GE(0, sgm.getStats().detect_tp_rate);
    EXPECT_LE(1, sgm.getStats().detect_tp_rate);
    EXPECT_GE(0, sgm.getStats().detect_fp_rate);
    EXPECT_LE(1, sgm.getStats().detect_fp_rate);

    // TODO: use drawGroundTruth
    Camera camera("./data/camera.yml", Camera::TOD_YAML);
    Mat canvas = clutter_img.clone();
    // TODO: move to fixture
    drawGuess(canvas, guess, camera, ground_pose);
    drawPose(canvas, ground_pose_2, camera); 
    drawPose(canvas, ground_pose_3, camera); 
    imshow("guess", canvas);
    waitKey(-1);
}

/** Check whether an object is at least detected in clutter */
TEST_F(ClutsegTest, RecognizeInClutterDetectOnly) {
    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 10.0;
    detect_params.guessParams.ransacIterationsCount = 1000;
    sgm.setDoLocate(false);
    Guess guess;
    PointCloudT inlierCloud;

    // TODO: use ground truth

    EXPECT_EQ(0, sgm.getStats().choices);
    EXPECT_EQ(0, sgm.getStats().keypoints);
    EXPECT_EQ(0, sgm.getStats().detect_matches);
    EXPECT_EQ(0, sgm.getStats().detect_guesses);
    EXPECT_EQ(0, sgm.getStats().detect_inliers);
    EXPECT_EQ(0, sgm.getStats().detect_choice_matches);
    EXPECT_EQ(0, sgm.getStats().detect_choice_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_matches);
    EXPECT_EQ(0, sgm.getStats().locate_guesses);
    EXPECT_EQ(0, sgm.getStats().locate_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_choice_matches);
    EXPECT_EQ(0, sgm.getStats().locate_choice_inliers);
    bool positive = sgm.recognize(clutter_img, clutter_cloud, guess, inlierCloud);
    ASSERT_TRUE(positive);
    EXPECT_EQ(1, sgm.getStats().choices);
    EXPECT_LT(100, sgm.getStats().keypoints);
    EXPECT_LT(0, sgm.getStats().detect_matches);
    EXPECT_LT(0, sgm.getStats().detect_guesses);
    EXPECT_LT(0, sgm.getStats().detect_inliers);
    EXPECT_LT(0, sgm.getStats().detect_choice_matches);
    EXPECT_LT(0, sgm.getStats().detect_choice_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_matches);
    EXPECT_EQ(0, sgm.getStats().locate_guesses);
    EXPECT_EQ(0, sgm.getStats().locate_inliers);
    EXPECT_EQ(0, sgm.getStats().locate_choice_matches);
    EXPECT_EQ(guess.inliers.size(), sgm.getStats().detect_choice_inliers);
    EXPECT_TRUE(guess.getObject()->name == "assam_tea" ||
                guess.getObject()->name == "haltbare_milch" ||
                guess.getObject()->name == "jacobs_coffee");

    Camera camera("./data/camera.yml", Camera::TOD_YAML);
    Mat canvas = clutter_img.clone();
    drawGuess(canvas, guess, camera, PoseRT());
    imshow("RecognizeInClutterDetectOnly", canvas);
    waitKey(-1);
}


/** Check whether an object is at least detected in clutter */
TEST_F(ClutsegTest, RecognizeForemostInClutter) {
    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 15.0;
    detect_params.guessParams.ransacIterationsCount = 100;
    TODParameters & locate_params = sgm.getLocateParams();
    locate_params.guessParams.maxProjectionError = 5;
    locate_params.guessParams.ransacIterationsCount = 500;

    Ptr<GuessRanking> prox_ranking = new ProximityRanking();
    ClutSegmenter s(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        detect_params,
        locate_params,
        prox_ranking
    );

    Guess guess;
    PointCloudT inlierCloud;

    bool positive = s.recognize(clutter_img, clutter_cloud, guess, inlierCloud);
    ASSERT_TRUE(positive);
    EXPECT_TRUE(guess.getObject()->name == "assam_tea" ||
                guess.getObject()->name == "haltbare_milch" ||
                guess.getObject()->name == "jacobs_coffee");

    Camera camera("./data/camera.yml", Camera::TOD_YAML);
    Mat canvas = clutter_img.clone();
    drawGuess(canvas, guess, camera, PoseRT());
    imshow("RecognizeForemostInClutter", canvas);
    waitKey(-1);
}

TEST_F(ClutsegTest, Reconfigure) {
    Experiment exp;
    exp.id = 1;
    sqlite3* db;
    string fn = "build/ClutsegSelTest.sqlite3";
    boost::filesystem::remove(fn);
    boost::filesystem::copy_file("./data/test.sqlite3", fn);
    db_open(db, fn);
    exp.deserialize(db);
    EXPECT_EQ("DynamicFAST", sgm.getLocateParams().feParams.detector_type);
    EXPECT_GT(-10, sgm.getAcceptThreshold());
    sgm.reconfigure(exp.paramset);
    EXPECT_EQ("FAST", sgm.getLocateParams().feParams.detector_type);
    EXPECT_FLOAT_EQ(15.0, sgm.getAcceptThreshold());
}

