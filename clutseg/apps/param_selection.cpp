/**
 * Author: Julius Adorf
 *
 * This application shall find good parameters for tod_training, tod_detecting
 * and clutseg. Basically, it does so by factorial design. Since the parameter
 * space is extremely large, the factors have to be carefully selected. Also,
 * results must be meticoulusly recorded.
 *
 * This application assumes that training set and test set are fixed. Results
 * are stored in a sqlite3 database.
 */

#include "clutseg/check.h"
#include "clutseg/db.h"
#include "clutseg/runner.h"
#include "clutseg/storage.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>


using namespace clutseg;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

/** Generates a prototype for an experiment setup that can be replicated,
 * adapted and written to the database (i.e. enqueuing the setup for being
 * carried out). The returned experiment setup is initialized to have some
 * reasonable defaults. */
Experiment createExperiment() {
    Experiment e;
    e.train_set = "ias_kinect_train_v2";
    e.test_set = "ias_kinect_test_grounded_21";
    e.paramset.pms_clutseg.accept_threshold = 10;
    e.paramset.pms_clutseg.ranking = "InliersRanking";
    e.paramset.train_pms_fe.detector_type = "FAST";
    e.paramset.train_pms_fe.extractor_type = "multi-scale";
    e.paramset.train_pms_fe.descriptor_type = "rBRIEF";
    e.paramset.train_pms_fe.detector_params["min_features"] = 500;
    e.paramset.train_pms_fe.detector_params["max_features"] = 700;
    e.paramset.train_pms_fe.detector_params["threshold"] = 25;
    e.paramset.train_pms_fe.extractor_params["octaves"] = 3;
    e.paramset.recog_pms_fe.detector_type = "FAST";
    e.paramset.recog_pms_fe.extractor_type = "multi-scale";
    e.paramset.recog_pms_fe.descriptor_type = "rBRIEF";
    e.paramset.recog_pms_fe.detector_params["min_features"] = 800;
    e.paramset.recog_pms_fe.detector_params["max_features"] = 1200;
    e.paramset.recog_pms_fe.detector_params["threshold"] = 30;
    e.paramset.recog_pms_fe.extractor_params["octaves"] = 3;
    e.paramset.detect_pms_match.type = "LSH-BINARY";
    e.paramset.detect_pms_match.knn = 3; 
    e.paramset.detect_pms_match.doRatioTest = true; 
    e.paramset.detect_pms_match.ratioThreshold= 0.8;
    e.paramset.detect_pms_guess.ransacIterationsCount = 1000; 
    e.paramset.detect_pms_guess.minInliersCount = 10; 
    e.paramset.detect_pms_guess.maxProjectionError = 15; 
    e.paramset.locate_pms_match.type = "LSH-BINARY";
    e.paramset.locate_pms_match.knn = 3; 
    e.paramset.locate_pms_match.doRatioTest = false; 
    e.paramset.locate_pms_match.ratioThreshold= 0;
    e.paramset.locate_pms_guess.ransacIterationsCount = 1000; 
    e.paramset.locate_pms_guess.minInliersCount = 15; 
    e.paramset.locate_pms_guess.maxProjectionError = 12; 
    return e;
}

void insert_if_not_exist(sqlite3* & db, Experiment & e) {
    sqlite3_stmt *read;
    db_prepare(db, read, "select name from experiment where name='"+e.name+"'");
    try {
        db_step(read, SQLITE_DONE); 
        e.serialize(db);
    } catch (...) {
        cout << "[PARAMSEL] Skipping the insertion of " + e.name + ", already exists." << endl;
    }
    sqlite3_finalize(read);
}

void insert_experiments(sqlite3* & db) {
    // FAST  + rBRIEF + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "single-fast-rbrief-lsh-binary";
        insert_if_not_exist(db, e);
    }
    // SIFT + rBRIEF + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "single-sift-rbrief-lsh-binary";
        e.paramset.train_pms_fe.detector_type = "SIFT";
        e.paramset.recog_pms_fe.detector_type = "SIFT";
        insert_if_not_exist(db, e);
    }
    // SURF + rBRIEF + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "single-surf-rbrief-lsh-binary";
        e.paramset.train_pms_fe.detector_type = "SURF";
        e.paramset.recog_pms_fe.detector_type = "SURF";
        insert_if_not_exist(db, e);
    }
    // STAR + rBRIEF + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "single-star-rbrief-lsh-binary";
        e.paramset.train_pms_fe.detector_type = "STAR";
        e.paramset.recog_pms_fe.detector_type = "STAR";
        insert_if_not_exist(db, e);
    }
    // ORB + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "single-orb-lsh-binary";
        e.paramset.train_pms_fe.detector_type = "ORB";
        e.paramset.train_pms_fe.extractor_type = "ORB";
        e.paramset.train_pms_fe.descriptor_type = "ORB";
        e.paramset.train_pms_fe.detector_params["threshold"] = 0.000001;
        e.paramset.recog_pms_fe.detector_type = "ORB";
        e.paramset.recog_pms_fe.extractor_type = "ORB";
        e.paramset.recog_pms_fe.descriptor_type = "ORB";
        e.paramset.recog_pms_fe.detector_params["threshold"] = 0.000001;
        insert_if_not_exist(db, e);
    }
}

ExperimentRunner runner;

void terminate(int s) {
    runner.terminate = true;
}

int main(int argc, char **argv) {
    // ... in order to convert it to a ROS node
    // #include <ros/ros.h>
    // #include <ros/console.h>
    // ros::init(argc, argv, "param_selection");
    // ros::NodeHandle n;

    if (argc != 4) {
        cerr << "Usage: param_selection <database> <train_cache> <result_dir>" << endl;
        return -1;
    }
    bfs::path db_path = argv[1];
    bfs::path cache_dir = argv[2];
    bfs::path result_dir = argv[3];

    assert_path_exists(db_path);
    assert_path_exists(cache_dir);
    assert_path_exists(result_dir);

    sqlite3* db;
    cout << "Opening database ..." << endl;
    db_open(db, db_path);
    cout << "Inserting experiment setups ..." << endl;
    insert_experiments(db);
    TrainFeaturesCache cache(cache_dir);
    ResultStorage storage(result_dir);
    cout << "Running experiments ..." << endl;
    runner = ExperimentRunner(db, cache, storage);

    // Use custom signal handler
    void (*prev_fn)(int);
    prev_fn = signal (SIGINT, terminate);
    if (prev_fn==SIG_IGN) signal (SIGINT,SIG_IGN);

    runner.run();
    db_close(db);
    return 0;

    // IDEA: it might be best to randomly choose a couple of images out of a
    // large testing set such to prevent having to test too many configurations
    // yet to get a too large bias.  in favor of certain parameter
    // configurations. So if several parameter configurations are tested that
    // are close to each other and one of the parameter configurations shows
    // extreme results compared to the others and we assume a somewhat
    // well-behaving problem, then we can notice such an outlier.
}

