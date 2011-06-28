/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/check.h"
#include "clutseg/clutseg.h"
#include "clutseg/common.h"
#include "clutseg/db.h"
#include "clutseg/experiment.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <gtest/gtest.h>
    #include <limits.h>
    #include <tod/detecting/Loader.h>
    #include <pcl/io/pcd_io.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <iostream>
    #include <stdlib.h>
#include "clutseg/gcc_diagnostic_enable.h"

#pragma GCC diagnostic ignored "-Wunused-parameter"

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

class test_clutseg : public ::testing::Test {

    public:

        virtual void SetUp() {
            if (!loaded) {
                FeatureExtractionParams fp;
                readFeParams("data/test_clutseg.features.config.yaml", fp);

                TODParameters dp;
                FileStorage dp_in("data/test_clutseg.detect.config.yaml", FileStorage::READ);
                dp.read(dp_in[TODParameters::YAML_NODE_NAME]);
                dp_in.release();

                TODParameters lp;
                FileStorage lp_in("data/test_clutseg.refine.config.yaml", FileStorage::READ);
                lp.read(lp_in[TODParameters::YAML_NODE_NAME]);
                lp_in.release();

                cache = TrainFeaturesCache("build/train_cache");
                tr_feat = TrainFeatures("ias_kinect_train_v2", fp);
                if (!fast() && !cache.trainFeaturesExist(tr_feat)) {
                    tr_feat.generate();
                    cache.addTrainFeatures(tr_feat);
               }
               sgm = Clutsegmenter(cache.trainFeaturesDir(tr_feat).string(), dp, lp);
            }
            loaded = true;

            if (!fast()) {
                sgm.setDoRefine(true);
                sgm.resetStats();               
                sgm.getRefineParams().matcherParams.doRatioTest = false;
            }

            haltbare_milch_train_img = imread("./data/image_00000.png");
            pcl::io::loadPCDFile("./data/cloud_00000.pcd", haltbare_milch_train_cloud);

            // this unfortunately requires the test set being present
            clutter_img = imread(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded_21/at_hm_jc/image_00022.png");
            pcl::io::loadPCDFile(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded_21/at_hm_jc/cloud_00022.pcd", clutter_cloud);
            readLabelSet(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_test_grounded_21/at_hm_jc/image_00022.png.ground.yaml", clutter_truth);
            assert(!clutter_truth.labels.empty());

            camera = Camera("./data/camera.yml", Camera::TOD_YAML);
        }

        static Clutsegmenter sgm;
        static bool loaded;
        TrainFeaturesCache cache;
        TrainFeatures tr_feat;
        Mat haltbare_milch_train_img;
        PointCloudT haltbare_milch_train_cloud;
        Mat clutter_img;
        PointCloudT clutter_cloud;
        LabelSet clutter_truth;
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
            EXPECT_EQ(0, sgm.getStats().acc_refine_matches);
            EXPECT_EQ(0, sgm.getStats().acc_refine_guesses);
            EXPECT_EQ(0, sgm.getStats().acc_refine_inliers);
            EXPECT_EQ(0, sgm.getStats().acc_refine_choice_matches);
            EXPECT_EQ(0, sgm.getStats().acc_refine_choice_inliers);
        }

        void check_postconditions() {
            EXPECT_EQ(1, sgm.getStats().choices);
            EXPECT_LT(100, sgm.getStats().acc_keypoints);
            EXPECT_LT(0, sgm.getStats().acc_detect_matches);
            EXPECT_LT(0, sgm.getStats().acc_detect_guesses);
            EXPECT_LT(0, sgm.getStats().acc_detect_inliers);
            EXPECT_LT(0, sgm.getStats().acc_detect_choice_matches);
            EXPECT_LT(0, sgm.getStats().acc_detect_choice_inliers);
            if (sgm.isDoRefine()) {
                EXPECT_LT(0, sgm.getStats().acc_refine_matches);
                EXPECT_LT(0, sgm.getStats().acc_refine_guesses);
                EXPECT_LT(0, sgm.getStats().acc_refine_inliers);
                EXPECT_LT(0, sgm.getStats().acc_refine_choice_matches);
                EXPECT_EQ(res.refine_choice.inliers.size(), sgm.getStats().acc_refine_choice_inliers);
            } else {
                EXPECT_EQ(0, sgm.getStats().acc_refine_matches);
                EXPECT_EQ(0, sgm.getStats().acc_refine_guesses);
                EXPECT_EQ(0, sgm.getStats().acc_refine_inliers);
                EXPECT_EQ(0, sgm.getStats().acc_refine_choice_matches);
                EXPECT_EQ(0, sgm.getStats().acc_refine_choice_inliers);
            }
            EXPECT_TRUE(clutter_truth.onScene(res.refine_choice.getObject()->name));
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
            ClutsegQuery query(clutter_img, clutter_cloud);
            sgm.recognize(query, res);
            ASSERT_TRUE(res.guess_made);
            check_postconditions();
            showGuessAndGroundTruth(test_name, res.refine_choice);
        }

};

Clutsegmenter test_clutseg::sgm;
bool test_clutseg::loaded;

/** Verify that changes to parameters do not affect existing Clutsegmenter
 * instances, i.e. that parameters are properly copied in constructor. */
TEST_F(test_clutseg, constructor_opaque) {
    SKIP_IF_FAST 

    TODParameters detect_params;
    TODParameters refine_params;

    Clutsegmenter s(
        cache.trainFeaturesDir(tr_feat).string(),
        detect_params,
        refine_params
    );
    
    EXPECT_NE("SIFT", detect_params.feParams.detector_type);
    detect_params.feParams.detector_type = "SIFT";
    EXPECT_NE("SIFT", s.getDetectParams().feParams.detector_type);
}

/** Check whether the params returned is actually a reference to the
 * Clutsegmenter's parameters and that changes shine through. In this way,
 * parameter configuration can be changed easily without having to reload the
 * training base. */
TEST_F(test_clutseg, change_params_online) {
    SKIP_IF_FAST 

    EXPECT_FALSE(sgm.getRefineParams().matcherParams.doRatioTest);
    TODParameters & params = sgm.getRefineParams();
    params.matcherParams.doRatioTest = true;
    EXPECT_TRUE(sgm.getRefineParams().matcherParams.doRatioTest);
}

/** Check whether detection works for a training image. This is expected to
 * return with a whole bunch of inliers since it is only a training image. */
TEST_F(test_clutseg, recog_haltbare_milch) {
    SKIP_IF_FAST 

    ClutsegQuery query(haltbare_milch_train_img, haltbare_milch_train_cloud);
    sgm.recognize(query, res);
    EXPECT_TRUE(res.guess_made);
    EXPECT_EQ("haltbare_milch", res.refine_choice.getObject()->name);
    cout << "detected: " << res.refine_choice.getObject()->name << endl;
    cout << "inliers:  " << res.refine_choice.inliers.size() << endl;
    EXPECT_GT(res.refine_choice.inliers.size(), 10);
}

/** Check whether loading a single training base works without failing */
TEST_F(test_clutseg, load_single_training_base) {
    SKIP_IF_FAST 

    vector<Ptr<TexturedObject> > objects;
    Loader loader(string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train_v2");
    loader.readTexturedObjects(objects);
    objects.erase(objects.begin() + 1, objects.end());
    EXPECT_EQ(1, objects.size());
    TrainingBase base(objects);
}

/** Check whether an object is at least detected in clutter */
TEST_F(test_clutseg, recog_in_clutter) {
    SKIP_IF_FAST 

    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 15.0;
    detect_params.guessParams.ransacIterationsCount = 100;
    TODParameters & refine_params = sgm.getRefineParams();
    refine_params.guessParams.maxProjectionError = 8;
    refine_params.guessParams.ransacIterationsCount = 500;
    refine_params.guessParams.minInliersCount = 50;
    //sgm.setAcceptThreshold(20);

    ASSERT_TRUE(sgm.isDoRefine());
    recognize("recog_in_clutter");
}

/** Check whether an object is at least detected in clutter */
TEST_F(test_clutseg, recog_in_clutter_detect_only) {
    SKIP_IF_FAST 

    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 10.0;
    detect_params.guessParams.ransacIterationsCount = 1000;

    sgm.setDoRefine(false);
    ASSERT_FALSE(sgm.isDoRefine());
    recognize("recog_in_clutter_detect_only");
}


/** Check whether an object is at least detected in clutter */
TEST_F(test_clutseg, recog_foremost_in_clutter) {
    SKIP_IF_FAST 

    TODParameters & detect_params = sgm.getDetectParams();
    detect_params.guessParams.maxProjectionError = 15.0;
    detect_params.guessParams.ransacIterationsCount = 100;
    TODParameters & refine_params = sgm.getRefineParams();
    refine_params.guessParams.maxProjectionError = 7;
    refine_params.guessParams.ransacIterationsCount = 1000;
    refine_params.guessParams.minInliersCount = 30;

    Ptr<GuessRanking> prox_ranking = new ProximityRanking();
    sgm = Clutsegmenter(
        cache.trainFeaturesDir(tr_feat).string(),
        detect_params,
        refine_params,
        prox_ranking
    );

    ASSERT_TRUE(sgm.isDoRefine());
    recognize("recog_foremost_in_clutter");
}

TEST_F(test_clutseg, Reconfigure) {
    SKIP_IF_FAST 

    Experiment exp;
    exp.id = 1;
    sqlite3* db;
    string fn = "build/test_clutseg.reconfigure.sqlite3";
    boost::filesystem::remove(fn);
    boost::filesystem::copy_file("./data/test.sqlite3", fn);
    db_open(db, fn);
    exp.deserialize(db);
    EXPECT_GT(-10, sgm.getAcceptThreshold());
    sgm.reconfigure(exp.paramset);
    EXPECT_EQ("FAST", sgm.getRefineParams().feParams.detector_type);
    EXPECT_FLOAT_EQ(15.0, sgm.getAcceptThreshold());
}

TEST_F(test_clutseg, recog_using_tar) {
    SKIP_IF_FAST

    string bd = "build/test_clutseg.recog_using_tar";
    // untar 
    assert(system(str(boost::format("mkdir -p %s") % bd).c_str()) == 0);
    assert(system(str(boost::format("tar xvf data/orb.tar -C %s") % bd).c_str()) == 0);
    assert_path_exists(bd);

    // TODO: check in one modelbase 
    sgm = Clutsegmenter(bd);
    recognize("recog_using_tar");
}
