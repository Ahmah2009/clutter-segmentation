/**
 * Author: Julius Adorf
 */

#include "test.h"
#include "clutseg/db.h"
#include "clutseg/paramsel.h"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <gtest/gtest.h>
#include <time.h>

using namespace clutseg;
using namespace std;

struct ParamSelTest : public ::testing::Test {

    virtual void SetUp() {
        string fn = "build/ParamSelTest.sqlite3";
        boost::filesystem::remove(fn);
        boost::filesystem::copy_file("./data/test.sqlite3", fn);
        db_open(db, fn);

        experiment.id = -1;
        experiment.time = "2011-01-02 20:12:23";
        experiment.train_set = "hypothetical_train_set";
        experiment.test_set = "hypothetical_test_set";
        experiment.paramset.pms_clutseg.accept_threshold = 15;
        experiment.paramset.pms_clutseg.ranking = "InliersRanking";
        experiment.paramset.train_pms_fe.detector_type = "FAST";
        experiment.paramset.train_pms_fe.extractor_type = "multi-scale";
        experiment.paramset.train_pms_fe.descriptor_type = "rBRIEF";
        experiment.paramset.train_pms_fe.detector_params["min_features"] = 0;
        experiment.paramset.train_pms_fe.detector_params["max_features"] = 0;
        experiment.paramset.train_pms_fe.detector_params["threshold"] = 0;
        experiment.paramset.recog_pms_fe.detector_type = "FAST";
        experiment.paramset.recog_pms_fe.extractor_type = "multi-scale";
        experiment.paramset.recog_pms_fe.descriptor_type = "rBRIEF";
        experiment.paramset.recog_pms_fe.detector_params["min_features"] = 0;
        experiment.paramset.recog_pms_fe.detector_params["max_features"] = 0;
        experiment.paramset.recog_pms_fe.detector_params["threshold"] = 0;
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
    }

    void TearDown() {
        db_close(db);
    }

    sqlite3* db;
    Experiment experiment;

};

TEST_F(ParamSelTest, response_read) {
    // Validate against data as given by data/test.sql
    Response r;
    r.id = 1;
    r.deserialize(db); 
    EXPECT_FLOAT_EQ(0.78, r.value);
}

TEST_F(ParamSelTest, response_update) {
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

TEST_F(ParamSelTest, response_write_read) {
    Response & orig = experiment.response;
    orig.serialize(db);
    Response rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.value, rest.value);
}

TEST_F(ParamSelTest, pms_clutseg_read) {
    // Validate against data as given by data/test.sql
    ClutsegParams p;
    p.id = 1;
    p.deserialize(db); 
    EXPECT_FLOAT_EQ(15.0, p.accept_threshold);
    EXPECT_EQ("InliersRanking", p.ranking);
}

TEST_F(ParamSelTest, pms_clutseg_update) {
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


TEST_F(ParamSelTest, pms_clutseg_write_read) {
    ClutsegParams & orig = experiment.paramset.pms_clutseg;
    orig.serialize(db);
    ClutsegParams rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.accept_threshold, rest.accept_threshold);
    EXPECT_EQ(orig.ranking, rest.ranking);
}

TEST_F(ParamSelTest, experiment_read) {
    // Validate against data as given by data/test.sql
    Experiment e;
    e.id = 1;
    e.deserialize(db); 
    EXPECT_EQ("train", e.train_set);
    EXPECT_EQ("test", e.test_set);
}

TEST_F(ParamSelTest, experiment_update) {
    Experiment e;
    e.id = 1;
    e.deserialize(db); 
    e.train_set = "train_v2";
    e.test_set = "test_v2";
    e.serialize(db);
    EXPECT_EQ(1, e.id);
    Experiment e2;
    e2.id = 1;
    e2.deserialize(db);
    EXPECT_EQ(e.train_set, e2.train_set);
    EXPECT_EQ(e.test_set, e2.test_set);
    EXPECT_EQ(1, e2.id);
}

TEST_F(ParamSelTest, experiment_write_read) {
    Experiment & orig = experiment;
    EXPECT_EQ(false, orig.run);
    // must set orig.run to have the response to be
    // serialized as well
    orig.run = true;
    orig.response.value = 13;
    orig.serialize(db);
    Experiment rest;
    rest.id = orig.id;
    rest.paramset.id = orig.paramset.id;
    rest.response.id = orig.response.id;
    rest.deserialize(db); 
    EXPECT_EQ(experiment.train_set, rest.train_set);
    EXPECT_EQ(experiment.test_set, rest.test_set);
    EXPECT_EQ(13, rest.response.value);
    EXPECT_EQ(true, rest.run);
}

TEST_F(ParamSelTest, select_experiments_not_run) {
    // We need to be able to find those experiments that have not been run
    // yet. These are candidates for being carried out next. 
    Experiment e1 = experiment;
    Experiment e2 = experiment;
    Experiment e3 = experiment;
    e1.run = true;
    e2.run = false;
    e3.run = false;
    e3.paramset.pms_clutseg.ranking = "ProximityRanking";
    e1.serialize(db);
    e2.serialize(db);
    e3.serialize(db);
    EXPECT_TRUE(e1.run);
    EXPECT_FALSE(e2.run);
    EXPECT_FALSE(e3.run);
    EXPECT_EQ(-1, e2.response.id);
    EXPECT_EQ(-1, e3.response.id);
    vector<Experiment> exps;
    selectExperimentsNotRun(db, exps);
    ASSERT_EQ(2, exps.size());
    EXPECT_EQ(-1, exps[0].response.id);
    EXPECT_EQ(-1, exps[1].response.id);
    EXPECT_FALSE(exps[0].run);
    EXPECT_FALSE(exps[1].run);
    EXPECT_TRUE((exps[0].id == e2.id) || (exps[1].id == e2.id));
    EXPECT_TRUE((exps[0].id == e3.id) || (exps[1].id == e3.id));
    EXPECT_TRUE((exps[0].id != e3.id) || exps[0].paramset.pms_clutseg.ranking == "ProximityRanking");
    EXPECT_TRUE((exps[0].id == e3.id) || exps[0].paramset.pms_clutseg.ranking != "ProximityRanking");
}

