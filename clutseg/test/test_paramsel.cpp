/**
 * Author: Julius Adorf
 */

#include "test.h"
#include "clutseg/db.h"
#include "clutseg/experiment.h"
#include "clutseg/paramsel.h"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <gtest/gtest.h>
#include <time.h>
#include <tod/training/feature_extraction.h>

using namespace clutseg;
using namespace std;
using namespace tod;

struct test_paramsel : public ::testing::Test {

    virtual void SetUp() {
        string fn = "build/test_paramsel.sqlite3";
        boost::filesystem::remove(fn);
        boost::filesystem::copy_file("./data/test.sqlite3", fn);
        db_open(db, fn);

        experiment.id = -1;
        experiment.name = "some-unique-experiment-name";
        experiment.time = "2011-01-02 20:12:23";
        experiment.train_set = "hypothetical_train_set";
        experiment.test_set = "hypothetical_test_set";
        experiment.human_note = "a note inserted by a human";
        experiment.machine_note = "a note made by the machine";
        experiment.batch = "some_group";
        experiment.paramset.pms_clutseg.accept_threshold = 15;
        experiment.paramset.pms_clutseg.ranking = "InliersRanking";
        experiment.paramset.train_pms_fe.detector_type = "FAST";
        experiment.paramset.train_pms_fe.extractor_type = "multi-scale";
        experiment.paramset.train_pms_fe.descriptor_type = "rBRIEF";
        experiment.paramset.train_pms_fe.detector_params["n_features"] = 5000;
        experiment.paramset.train_pms_fe.detector_params["min_features"] = 0;
        experiment.paramset.train_pms_fe.detector_params["max_features"] = 0;
        experiment.paramset.train_pms_fe.detector_params["threshold"] = 0;
        experiment.paramset.train_pms_fe.extractor_params["scale_factor"] = 1.2f;
        experiment.paramset.train_pms_fe.extractor_params["octaves"] = 3;
        experiment.paramset.recog_pms_fe.detector_type = "FAST";
        experiment.paramset.recog_pms_fe.extractor_type = "multi-scale";
        experiment.paramset.recog_pms_fe.descriptor_type = "rBRIEF";
        experiment.paramset.recog_pms_fe.detector_params["n_features"] = 5000;
        experiment.paramset.recog_pms_fe.detector_params["min_features"] = 0;
        experiment.paramset.recog_pms_fe.detector_params["max_features"] = 0;
        experiment.paramset.recog_pms_fe.detector_params["threshold"] = 0;
        experiment.paramset.recog_pms_fe.extractor_params["scale_factor"] = 1.2f;
        experiment.paramset.recog_pms_fe.extractor_params["octaves"] = 3;
        experiment.paramset.detect_pms_match.type = "LSH-BINARY";
        experiment.paramset.detect_pms_match.knn = 3; 
        experiment.paramset.detect_pms_match.doRatioTest = true; 
        experiment.paramset.detect_pms_match.ratioThreshold= 0.8;
        experiment.paramset.detect_pms_guess.ransacIterationsCount = 100; 
        experiment.paramset.detect_pms_guess.minInliersCount = 5; 
        experiment.paramset.detect_pms_guess.maxProjectionError = 12; 
        experiment.paramset.locate_pms_match.type = "LSH-BINARY";
        experiment.paramset.locate_pms_match.knn = 3; 
        experiment.paramset.locate_pms_match.doRatioTest = false; 
        experiment.paramset.locate_pms_match.ratioThreshold= 0;
        experiment.paramset.locate_pms_guess.ransacIterationsCount = 100; 
        experiment.paramset.locate_pms_guess.minInliersCount = 5; 
        experiment.paramset.locate_pms_guess.maxProjectionError = 12; 
        experiment.response.value = 0.87;
        experiment.response.locate_sipc.rscore = 0.25;
        experiment.response.locate_sipc.tscore = 0.75;
        experiment.response.locate_sipc.cscore = 1.0;
        experiment.response.detect_sipc.acc_score = 40;
        experiment.response.detect_sipc.objects = 63;
        experiment.response.avg_angle_err = 0.34;
        experiment.response.avg_succ_angle_err = 0.08;
        experiment.response.avg_trans_err = 0.12;
        experiment.response.avg_succ_trans_err = 0.02;
        experiment.response.avg_angle_sq_err = 0.56;
        experiment.response.avg_succ_angle_sq_err = 0.15;
        experiment.response.avg_trans_sq_err = 0.53;
        experiment.response.avg_succ_trans_sq_err = 0.03;
        experiment.response.succ_rate = 0.63;
        experiment.response.mislabel_rate = 0.05;
        experiment.response.none_rate = 0.15;
        experiment.response.avg_keypoints = 913.0;
        experiment.response.avg_detect_guesses = 55;
        experiment.response.avg_detect_matches = 652.3;
        experiment.response.avg_detect_inliers = 9.2;
        experiment.response.avg_detect_choice_matches = 211.9;
        experiment.response.avg_detect_choice_inliers = 13.3;
        experiment.response.detect_tp = 35;
        experiment.response.detect_fp = 5;
        experiment.response.detect_fn = 10;
        experiment.response.detect_tn = 40;
        experiment.response.avg_locate_guesses = 33;
        experiment.response.avg_locate_matches = 802.1;
        experiment.response.avg_locate_inliers = 29.8;
        experiment.response.avg_locate_choice_matches = 802.1;
        experiment.response.avg_locate_choice_inliers = 39.8;
        experiment.response.train_runtime = 320.5;
        experiment.response.test_runtime = 214.8;
        experiment.record_commit();
    }

    void TearDown() {
        db_close(db);
    }

    sqlite3* db;
    Experiment experiment;

};

TEST_F(test_paramsel, init) {
    Experiment exp;
    EXPECT_EQ("", exp.name);
    EXPECT_EQ("", exp.human_note);
    EXPECT_EQ("", exp.machine_note);
    EXPECT_FLOAT_EQ(0, exp.response.value);
    EXPECT_FLOAT_EQ(0, exp.response.locate_sipc.rscore);
    EXPECT_FLOAT_EQ(0, exp.response.locate_sipc.tscore);
    EXPECT_FLOAT_EQ(0, exp.response.locate_sipc.cscore);
    EXPECT_FLOAT_EQ(0, exp.response.avg_angle_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_succ_angle_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_trans_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_succ_trans_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_angle_sq_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_succ_angle_sq_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_trans_sq_err);
    EXPECT_FLOAT_EQ(0, exp.response.avg_succ_trans_sq_err);
    EXPECT_FLOAT_EQ(0, exp.response.succ_rate);
    EXPECT_FLOAT_EQ(0, exp.response.mislabel_rate);
    EXPECT_FLOAT_EQ(0, exp.response.none_rate);
    EXPECT_FLOAT_EQ(0, exp.response.avg_keypoints);
    EXPECT_FLOAT_EQ(0, exp.response.avg_detect_guesses);
    EXPECT_FLOAT_EQ(0, exp.response.avg_detect_matches);
    EXPECT_FLOAT_EQ(0, exp.response.avg_detect_inliers);
    EXPECT_FLOAT_EQ(0, exp.response.avg_detect_choice_matches);
    EXPECT_FLOAT_EQ(0, exp.response.avg_detect_choice_inliers);
    EXPECT_FLOAT_EQ(0, exp.response.avg_locate_guesses);
    EXPECT_FLOAT_EQ(0, exp.response.avg_locate_matches);
    EXPECT_FLOAT_EQ(0, exp.response.avg_locate_inliers);
    EXPECT_FLOAT_EQ(0, exp.response.avg_locate_choice_matches);
    EXPECT_FLOAT_EQ(0, exp.response.avg_locate_choice_inliers);
    EXPECT_FLOAT_EQ(0, exp.response.train_runtime);
    EXPECT_FLOAT_EQ(0, exp.response.test_runtime);

    Response r;
    EXPECT_FLOAT_EQ(0, r.value);
    EXPECT_FLOAT_EQ(0, r.locate_sipc.rscore);
    EXPECT_FLOAT_EQ(0, r.locate_sipc.tscore);
    EXPECT_FLOAT_EQ(0, r.locate_sipc.cscore);
    EXPECT_FLOAT_EQ(0, r.avg_angle_err);
    EXPECT_FLOAT_EQ(0, r.avg_succ_angle_err);
    EXPECT_FLOAT_EQ(0, r.avg_trans_err);
    EXPECT_FLOAT_EQ(0, r.avg_succ_trans_err);
    EXPECT_FLOAT_EQ(0, r.avg_angle_sq_err);
    EXPECT_FLOAT_EQ(0, r.avg_succ_angle_sq_err);
    EXPECT_FLOAT_EQ(0, r.avg_trans_sq_err);
    EXPECT_FLOAT_EQ(0, r.avg_succ_trans_sq_err);
    EXPECT_FLOAT_EQ(0, r.succ_rate);
    EXPECT_FLOAT_EQ(0, r.mislabel_rate);
    EXPECT_FLOAT_EQ(0, r.none_rate);
    EXPECT_FLOAT_EQ(0, r.avg_keypoints);
    EXPECT_FLOAT_EQ(0, r.avg_detect_guesses);
    EXPECT_FLOAT_EQ(0, r.avg_detect_matches);
    EXPECT_FLOAT_EQ(0, r.avg_detect_inliers);
    EXPECT_FLOAT_EQ(0, r.avg_detect_choice_matches);
    EXPECT_FLOAT_EQ(0, r.avg_detect_choice_inliers);
    EXPECT_FLOAT_EQ(0, r.avg_locate_guesses);
    EXPECT_FLOAT_EQ(0, r.avg_locate_matches);
    EXPECT_FLOAT_EQ(0, r.avg_locate_inliers);
    EXPECT_FLOAT_EQ(0, r.avg_locate_choice_matches);
    EXPECT_FLOAT_EQ(0, r.avg_locate_choice_inliers);
    EXPECT_FLOAT_EQ(0, r.train_runtime);
    EXPECT_FLOAT_EQ(0, r.test_runtime);
}

TEST_F(test_paramsel, record_commit) {
    Experiment e;
    EXPECT_EQ(0, e.vcs_commit.size());
    e.record_commit();
    EXPECT_EQ(40, e.vcs_commit.size());
}

TEST_F(test_paramsel, response_read) {
    // Validate against data as given by data/test.sql
    Response r;
    r.id = 1;
    r.deserialize(db); 
    EXPECT_FLOAT_EQ(0.78, r.value);
}

TEST_F(test_paramsel, response_update) {
    Response r;
    r.id = 1;
    r.deserialize(db); 
    r.value = 2.0;
    r.serialize(db);
    EXPECT_EQ(1, r.id);
    Response r2;
    r2.id = 1;
    r2.deserialize(db);
    EXPECT_EQ(r.value, r2.value);
    EXPECT_EQ(1, r2.id);
}

TEST_F(test_paramsel, response_write_read) {
    Response & orig = experiment.response;
    orig.serialize(db);
    Response rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.value, rest.value);
    EXPECT_FLOAT_EQ(orig.detect_sipc.acc_score, rest.detect_sipc.acc_score);
    EXPECT_EQ(orig.detect_sipc.objects, rest.detect_sipc.objects);
    EXPECT_FLOAT_EQ(orig.detect_sipc.score(), rest.detect_sipc.score());
    EXPECT_FLOAT_EQ(orig.locate_sipc.rscore, rest.locate_sipc.rscore);
    EXPECT_FLOAT_EQ(orig.locate_sipc.tscore, rest.locate_sipc.tscore);
    EXPECT_FLOAT_EQ(orig.locate_sipc.cscore, rest.locate_sipc.cscore);
    EXPECT_FLOAT_EQ(orig.locate_sipc.score(), rest.locate_sipc.score());
    EXPECT_FLOAT_EQ(orig.avg_angle_err, rest.avg_angle_err);
    EXPECT_FLOAT_EQ(orig.avg_succ_angle_err, rest.avg_succ_angle_err);
    EXPECT_FLOAT_EQ(orig.avg_trans_err, rest.avg_trans_err);
    EXPECT_FLOAT_EQ(orig.avg_succ_trans_err, rest.avg_succ_trans_err);
    EXPECT_FLOAT_EQ(orig.avg_angle_sq_err, rest.avg_angle_sq_err);
    EXPECT_FLOAT_EQ(orig.avg_succ_angle_sq_err, rest.avg_succ_angle_sq_err);
    EXPECT_FLOAT_EQ(orig.avg_trans_sq_err, rest.avg_trans_sq_err);
    EXPECT_FLOAT_EQ(orig.avg_succ_trans_sq_err, rest.avg_succ_trans_sq_err);
    EXPECT_FLOAT_EQ(orig.succ_rate, rest.succ_rate);
    EXPECT_FLOAT_EQ(orig.mislabel_rate, rest.mislabel_rate);
    EXPECT_FLOAT_EQ(orig.none_rate, rest.none_rate);
    EXPECT_FLOAT_EQ(orig.avg_keypoints, rest.avg_keypoints);
    EXPECT_FLOAT_EQ(orig.avg_detect_guesses, rest.avg_detect_guesses);
    EXPECT_FLOAT_EQ(orig.avg_detect_matches, rest.avg_detect_matches);
    EXPECT_FLOAT_EQ(orig.avg_detect_inliers, rest.avg_detect_inliers);
    EXPECT_FLOAT_EQ(orig.avg_detect_choice_matches, rest.avg_detect_choice_matches);
    EXPECT_FLOAT_EQ(orig.avg_detect_choice_inliers, rest.avg_detect_choice_inliers);
    EXPECT_FLOAT_EQ(orig.detect_tp, rest.detect_tp);
    EXPECT_FLOAT_EQ(orig.detect_fp, rest.detect_fp);
    EXPECT_FLOAT_EQ(orig.detect_fn, rest.detect_fn);
    EXPECT_FLOAT_EQ(orig.detect_tn, rest.detect_tn);
    EXPECT_FLOAT_EQ(orig.avg_locate_guesses, rest.avg_locate_guesses);
    EXPECT_FLOAT_EQ(orig.avg_locate_matches, rest.avg_locate_matches);
    EXPECT_FLOAT_EQ(orig.avg_locate_inliers, rest.avg_locate_inliers);
    EXPECT_FLOAT_EQ(orig.avg_locate_choice_matches, rest.avg_locate_choice_matches);
    EXPECT_FLOAT_EQ(orig.avg_locate_choice_inliers, rest.avg_locate_choice_inliers);
    EXPECT_FLOAT_EQ(orig.train_runtime, rest.train_runtime);
    EXPECT_FLOAT_EQ(orig.test_runtime, rest.test_runtime);
}

TEST_F(test_paramsel, response_detach) {
    experiment.response.detach();
    EXPECT_EQ(-1, experiment.response.id);
}

TEST_F(test_paramsel, pms_fe_read) {
    // Validate against data as given by data/test.sql
    FeatureExtractionParams feParams;
    int64_t id = 1;
    deserialize_pms_fe(db, feParams, id); 
    EXPECT_EQ("FAST", feParams.detector_type);
    EXPECT_EQ("multi-scale", feParams.extractor_type);
    EXPECT_EQ("rBRIEF", feParams.descriptor_type);
    EXPECT_EQ(3, feParams.extractor_params["octaves"]);
    EXPECT_FLOAT_EQ(1.2, feParams.extractor_params["scale_factor"]);
    EXPECT_FLOAT_EQ(40, feParams.detector_params["threshold"]);
    EXPECT_FLOAT_EQ(0, feParams.detector_params["min_features"]);
    EXPECT_FLOAT_EQ(0, feParams.detector_params["max_features"]);
    EXPECT_FLOAT_EQ(5000, feParams.detector_params["n_features"]);
}

TEST_F(test_paramsel, pms_fe_update) {
    FeatureExtractionParams feParams;
    int64_t id = 1;
    deserialize_pms_fe(db, feParams, id); 
    feParams.detector_type = "STAR";
    feParams.detector_params["n_features"] = 4000;
    feParams.detector_params["scale_factor"] = 1.2;
    serialize_pms_fe(db, feParams, id);
    FeatureExtractionParams feParams2;
    deserialize_pms_fe(db, feParams2, id);
    EXPECT_EQ(feParams.detector_type, feParams2.detector_type);
    EXPECT_EQ("STAR", feParams2.detector_type);
    EXPECT_FLOAT_EQ(4000, feParams.detector_params["n_features"]);
    EXPECT_FLOAT_EQ(1.2, feParams.detector_params["scale_factor"]);
}

TEST_F(test_paramsel, pms_fe_write_read) {
    Response & orig = experiment.response;
    orig.serialize(db);
    Response rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.value, rest.value);
}

TEST_F(test_paramsel, pms_clutseg_read) {
    // Validate against data as given by data/test.sql
    ClutsegParams p;
    p.id = 1;
    p.deserialize(db); 
    EXPECT_FLOAT_EQ(15.0, p.accept_threshold);
    EXPECT_EQ("InliersRanking", p.ranking);
}

TEST_F(test_paramsel, pms_clutseg_update) {
    ClutsegParams p;
    p.id = 1;
    p.deserialize(db); 
    p.accept_threshold = 30.5;
    p.ranking = "ProximityRanking";
    p.serialize(db);
    EXPECT_EQ(1, p.id);
    ClutsegParams p2;
    p2.id = 1;
    p2.deserialize(db);
    EXPECT_EQ(p.accept_threshold, p2.accept_threshold);
    EXPECT_EQ(p.ranking, p2.ranking);
    EXPECT_EQ(1, p2.id);
}

TEST_F(test_paramsel, pms_clutseg_write_read) {
    ClutsegParams & orig = experiment.paramset.pms_clutseg;
    orig.serialize(db);
    ClutsegParams rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.accept_threshold, rest.accept_threshold);
    EXPECT_EQ(orig.ranking, rest.ranking);
}

TEST_F(test_paramsel, pms_clutseg_detach) {
    experiment.paramset.pms_clutseg.detach();
    EXPECT_EQ(-1, experiment.paramset.pms_clutseg.id);
}

TEST_F(test_paramsel, experiment_read) {
    // Validate against data as given by data/test.sql
    Experiment e;
    e.id = 1;
    e.deserialize(db); 
    EXPECT_EQ("some-experiment", e.name);
    EXPECT_EQ("train", e.train_set);
    EXPECT_EQ("test", e.test_set);
}

TEST_F(test_paramsel, experiment_update) {
    Experiment e;
    e.id = 1;
    e.deserialize(db); 
    e.train_set = "train_v2";
    e.test_set = "test_v2";
    e.vcs_commit = "aaaaa1d7307ef27a65ab82f297be80390b5ccccc";
    e.serialize(db);
    EXPECT_EQ(1, e.id);
    Experiment e2;
    e2.id = 1;
    e2.deserialize(db);
    EXPECT_EQ(e.train_set, e2.train_set);
    EXPECT_EQ(e.test_set, e2.test_set);
    EXPECT_EQ(e.vcs_commit, e2.vcs_commit);
    EXPECT_EQ(e.batch, e2.batch);
    EXPECT_EQ(e.skip, e2.skip);
    EXPECT_EQ(e.flags, e2.flags);
    EXPECT_EQ(1, e2.id);
}

TEST_F(test_paramsel, experiment_write_read) {
    Experiment & orig = experiment;
    EXPECT_EQ(false, orig.has_run);
    // must set orig.has_run to have the response to be
    // serialized as well
    orig.has_run = true;
    orig.response.value = 13;
    orig.serialize(db);
    Experiment rest;
    rest.id = orig.id;
    rest.paramset.id = orig.paramset.id;
    rest.response.id = orig.response.id;
    rest.deserialize(db); 
    EXPECT_EQ(experiment.train_set, rest.train_set);
    EXPECT_EQ(experiment.test_set, rest.test_set);
    EXPECT_EQ(experiment.human_note, rest.human_note);
    EXPECT_EQ(experiment.machine_note, rest.machine_note);
    EXPECT_EQ(experiment.batch, rest.batch);
    EXPECT_EQ(experiment.skip, rest.skip);
    EXPECT_EQ(experiment.flags, rest.flags);
    EXPECT_EQ(13, rest.response.value);
    EXPECT_EQ(true, rest.has_run);
}

TEST_F(test_paramsel, experiment_detach) {
    experiment.detach();
    EXPECT_EQ(-1, experiment.response.id);
    EXPECT_EQ(-1, experiment.paramset.pms_clutseg.id);
    EXPECT_EQ(-1, experiment.paramset.train_pms_fe_id);
    EXPECT_EQ(-1, experiment.paramset.recog_pms_fe_id);
    EXPECT_EQ(-1, experiment.paramset.detect_pms_match_id);
    EXPECT_EQ(-1, experiment.paramset.detect_pms_guess_id);
    EXPECT_EQ(-1, experiment.paramset.locate_pms_match_id);
    EXPECT_EQ(-1, experiment.paramset.locate_pms_guess_id);
}

TEST_F(test_paramsel, experiment_vcs_commit) {
    Experiment exp;
    exp.id = 1;
    exp.deserialize(db);
    EXPECT_EQ("ccb521d7307ef27a65ab82f297be80390b5599bb", exp.vcs_commit);
}

TEST_F(test_paramsel, experiment_flags) {
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_VALID);
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_INVALID);
    experiment.flags |= Experiment::FLAG_FEPARAMS_VALID; 
    EXPECT_TRUE(experiment.flags & Experiment::FLAG_FEPARAMS_VALID);
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_INVALID);
    experiment.flags |= Experiment::FLAG_FEPARAMS_INVALID; 
    EXPECT_TRUE(experiment.flags & Experiment::FLAG_FEPARAMS_VALID);
    EXPECT_TRUE(experiment.flags & Experiment::FLAG_FEPARAMS_INVALID);
    experiment.flags &= ~Experiment::FLAG_FEPARAMS_VALID;
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_VALID);
    EXPECT_TRUE(experiment.flags & Experiment::FLAG_FEPARAMS_INVALID);
    experiment.flags &= ~Experiment::FLAG_FEPARAMS_INVALID;
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_VALID);
    EXPECT_FALSE(experiment.flags & Experiment::FLAG_FEPARAMS_INVALID);
}

TEST_F(test_paramsel, select_experiments_not_run) {
    // We need to be able to find those experiments that have not been run
    // yet. These are candidates for being carried out next. 
    Experiment e1 = experiment;
    Experiment e2 = experiment;
    Experiment e3 = experiment;
    e1.has_run = true;
    e2.has_run = false;
    e3.has_run = false;
    e1.name = "e1";
    e2.name = "e2";
    e3.name = "e3";
    e3.paramset.pms_clutseg.ranking = "ProximityRanking";
    e1.serialize(db);
    e2.serialize(db);
    e3.serialize(db);
    EXPECT_TRUE(e1.has_run);
    EXPECT_FALSE(e2.has_run);
    EXPECT_FALSE(e3.has_run);
    EXPECT_EQ(-1, e2.response.id);
    EXPECT_EQ(-1, e3.response.id);
    vector<Experiment> exps;
    selectExperimentsNotRun(db, exps);
    ASSERT_EQ(2, exps.size());
    EXPECT_EQ(-1, exps[0].response.id);
    EXPECT_EQ(-1, exps[1].response.id);
    EXPECT_FALSE(exps[0].has_run);
    EXPECT_FALSE(exps[1].has_run);
    EXPECT_TRUE((exps[0].id == e2.id) || (exps[1].id == e2.id));
    EXPECT_TRUE((exps[0].id == e3.id) || (exps[1].id == e3.id));
    EXPECT_TRUE((exps[0].id != e3.id) || exps[0].paramset.pms_clutseg.ranking == "ProximityRanking");
    EXPECT_TRUE((exps[0].id == e3.id) || exps[0].paramset.pms_clutseg.ranking != "ProximityRanking");
}

TEST_F(test_paramsel, sort_experiments_by_train_features) {
    Experiment e1 = experiment;
    Experiment e2 = experiment;
    Experiment e3 = experiment;
    e2.paramset.train_pms_fe = FeatureExtractionParams::CreateSampleParams();
    e1.name = "e1";
    e2.name = "e2";
    e3.name = "e3";
    e1.serialize(db);
    e2.serialize(db);
    e3.serialize(db);
    vector<Experiment> exps;
    selectExperimentsNotRun(db, exps);
    sortExperimentsByTrainFeatures(exps);
    ASSERT_EQ(3, exps.size());
    EXPECT_TRUE(exps[0].id == e2.id || exps[2].id == e2.id);
}

TEST_F(test_paramsel, to_detect_tod_parameters) {
    TODParameters dp = experiment.paramset.toDetectTodParameters();
    EXPECT_EQ(experiment.paramset.detect_pms_guess.minInliersCount, 5);
    EXPECT_EQ(dp.guessParams.minInliersCount, 5);
    dp.guessParams.minInliersCount = 10;
    EXPECT_EQ(dp.guessParams.minInliersCount, 10);
    EXPECT_EQ(experiment.paramset.detect_pms_guess.minInliersCount, 5);
    experiment.paramset.detect_pms_guess.minInliersCount = 15;
    EXPECT_EQ(dp.guessParams.minInliersCount, 10);
    EXPECT_EQ(experiment.paramset.detect_pms_guess.minInliersCount, 15);
}
