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
    experiment.train_set = "ias_kinect_train_v2";
    experiment.test_set = "ias_kinect_test_grounded_21";
    experiment.paramset.pms_clutseg.accept_threshold = 10;
    experiment.paramset.pms_clutseg.ranking = "InliersRanking";
    experiment.paramset.train_pms_fe.detector_type = "FAST";
    experiment.paramset.train_pms_fe.extractor_type = "multi-scale";
    experiment.paramset.train_pms_fe.descriptor_type = "rBRIEF";
    experiment.paramset.train_pms_fe.detector_params["min_features"] = 0;
    experiment.paramset.train_pms_fe.detector_params["max_features"] = 0;
    experiment.paramset.train_pms_fe.detector_params["threshold"] = 25;
    experiment.paramset.train_pms_fe.extractor_params["octaves"] = 3;
    experiment.paramset.recog_pms_fe.detector_type = "FAST";
    experiment.paramset.recog_pms_fe.extractor_type = "multi-scale";
    experiment.paramset.recog_pms_fe.descriptor_type = "rBRIEF";
    experiment.paramset.recog_pms_fe.detector_params["min_features"] = 0;
    experiment.paramset.recog_pms_fe.detector_params["max_features"] = 0;
    experiment.paramset.recog_pms_fe.detector_params["threshold"] = 30;
    experiment.paramset.recog_pms_fe.extractor_params["octaves"] = 3;
    experiment.paramset.detect_pms_match.type = "LSH-BINARY";
    experiment.paramset.detect_pms_match.knn = 3; 
    experiment.paramset.detect_pms_match.doRatioTest = true; 
    experiment.paramset.detect_pms_match.ratioThreshold= 0.8;
    experiment.paramset.detect_pms_guess.ransacIterationsCount = 1000; 
    experiment.paramset.detect_pms_guess.minInliersCount = 10; 
    experiment.paramset.detect_pms_guess.maxProjectionError = 15; 
    experiment.paramset.locate_pms_match.type = "LSH-BINARY";
    experiment.paramset.locate_pms_match.knn = 3; 
    experiment.paramset.locate_pms_match.doRatioTest = false; 
    experiment.paramset.locate_pms_match.ratioThreshold= 0;
    experiment.paramset.locate_pms_guess.ransacIterationsCount = 1000; 
    experiment.paramset.locate_pms_guess.minInliersCount = 15; 
    experiment.paramset.locate_pms_guess.maxProjectionError = 12; 
    return e;
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
    TrainFeaturesCache cache(cache_dir);
    ResultStorage storage(result_dir);
    cout << "Running experiments ..." << endl;
    ExperimentRunner runner(db, cache, storage);
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

