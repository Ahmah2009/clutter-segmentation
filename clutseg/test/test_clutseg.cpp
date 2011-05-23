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

            clutter_truth.read(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/image_00000.png.ground.yaml");
            assert(!clutter_truth.labels.empty());

            camera = Camera("./data/camera.yml", Camera::TOD_YAML);
        }

        static ClutSegmenter sgm;
        static bool loaded;
        Mat haltbare_milch_train_img;
        PointCloudT haltbare_milch_train_cloud;
        Mat clutter_img;
        PointCloudT clutter_cloud;
        GroundTruth clutter_truth;
        Camera camera;
        Result res;

        void check_preconditions() {
            EXPECT_FALSE(clutter_truth.labels.empty());
            EXPECT_EQ(0, sgm.getStats().choices);
            EXPECT_EQ(0, sgm.getStats().acc_keypoints);
            EXPECT_EQ(0, sgm.getStats().acc_detect_matches);
            EXPECT_EQ(0, sgm.getStats().acc_detect_guesses);
            EXPECT_EQ(0, sgm.getStats().acc_detect_inliers);
            EXPECT_EQ(0, sgm.getStats().acc_detect_choice_matches);
            EXPECT_EQ(0, sgm.getStats().acc_detect_choice_inliers);
            EXPECT_EQ(0, sgm.getStats().acc_locate_matches);
            EXPECT_EQ(0, sgm.getStats().acc_locate_guesses);
            EXPECT_EQ(0, sgm.getStats().acc_locate_inliers);
            EXPECT_EQ(0, sgm.getStats().acc_locate_choice_matches);
            EXPECT_EQ(0, sgm.getStats().acc_locate_choice_inliers);
        }

        void check_postconditions() {
            EXPECT_EQ(1, sgm.getStats().choices);
            EXPECT_LT(100, sgm.getStats().acc_keypoints);
            EXPECT_LT(0, sgm.getStats().acc_detect_matches);
            EXPECT_LT(0, sgm.getStats().acc_detect_guesses);
            EXPECT_LT(0, sgm.getStats().acc_detect_inliers);
            EXPECT_LT(0, sgm.getStats().acc_detect_choice_matches);
            EXPECT_LT(0, sgm.getStats().acc_detect_choice_inliers);
            if (sgm.isDoLocate()) {
                EXPECT_LT(0, sgm.getStats().acc_locate_matches);
                EXPECT_LT(0, sgm.getStats().acc_locate_guesses);
                EXPECT_LT(0, sgm.getStats().acc_locate_inliers);
                EXPECT_LT(0, sgm.getStats().acc_locate_choice_matches);
                EXPECT_EQ(res.locate_choice.inliers.size(), sgm.getStats().acc_locate_choice_inliers);
            } else {
                EXPECT_EQ(0, sgm.getStats().acc_locate_matches);
                EXPECT_EQ(0, sgm.getStats().acc_locate_guesses);
                EXPECT_EQ(0, sgm.getStats().acc_locate_inliers);
                EXPECT_EQ(0, sgm.getStats().acc_locate_choice_matches);
                EXPECT_EQ(0, sgm.getStats().acc_locate_choice_inliers);
            }
            EXPECT_TRUE(clutter_truth.onScene(res.locate_choice.getObject()->name));
        }

        void showGuessAndGroundTruth(const string & test_name, const Guess & choice) {
            Mat c = clutter_img.clone();
            drawGroundTruth(c, clutter_truth, camera);
            drawGuess(c, choice, camera, PoseRT());
            imshow(test_name, c);
            waitKey(-1);
        }

        void recognize(const string & test_name) {
            check_preconditions();
            bool positive = sgm.recognize(clutter_img, clutter_cloud, res);
            ASSERT_TRUE(positive);
            check_postconditions();
            showGuessAndGroundTruth(test_name, res.locate_choice);
        }

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
    bool positive = sgm.recognize(haltbare_milch_train_img,
                                        haltbare_milch_train_cloud,
                                        res);
    EXPECT_TRUE(positive);
    EXPECT_EQ("haltbare_milch", res.locate_choice.getObject()->name);
    cout << "detected: " << res.locate_choice.getObject()->name << endl;
    cout << "inliers:  " << res.locate_choice.inliers.size() << endl;
    EXPECT_GT(res.locate_choice.inliers.size(), 500);
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

    ASSERT_TRUE(sgm.isDoLocate());
    recognize("RecognizeInClutter");
}

/** Check whether an object is at least detected in clutter */
TEST_F(ClutsegTest, RecognizeInClutterDetectOnly) {
    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 10.0;
    detect_params.guessParams.ransacIterationsCount = 1000;

    sgm.setDoLocate(false);
    ASSERT_FALSE(sgm.isDoLocate());
    recognize("RecognizeInClutterDetectOnly");
}


/** Check whether an object is at least detected in clutter */
TEST_F(ClutsegTest, RecognizeForemostInClutter) {
    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 15.0;
    detect_params.guessParams.ransacIterationsCount = 100;
    TODParameters & locate_params = sgm.getLocateParams();
    locate_params.guessParams.maxProjectionError = 7;
    locate_params.guessParams.ransacIterationsCount = 1000;
    locate_params.guessParams.minInliersCount = 30;

    Ptr<GuessRanking> prox_ranking = new ProximityRanking();
    sgm = ClutSegmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        detect_params,
        locate_params,
        prox_ranking
    );

    ASSERT_TRUE(sgm.isDoLocate());
    recognize("RecognizeForemostInClutter");
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

