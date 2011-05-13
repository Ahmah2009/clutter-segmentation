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
        string fn = "build/test.sqlite3";
        boost::filesystem::remove(fn);
        boost::filesystem::copy_file("./data/test.sqlite3", fn);
        db_open(db, fn);

        time(&experiment.time);
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

TEST_F(ParamSelTest, pms_clutseg_write_read) {
    ClutsegParams & orig = experiment.paramset.pms_clutseg;
    orig.serialize(db);
    ClutsegParams rest;
    rest.id = orig.id;
    rest.deserialize(db); 
    EXPECT_FLOAT_EQ(orig.accept_threshold, rest.accept_threshold);
    EXPECT_EQ(orig.ranking, rest.ranking);
}

