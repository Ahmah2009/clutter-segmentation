/**
 * Author: Julius Adorf
 *
 * Feeds the experiment runner with new experiments.
 */

#include "clutseg/db.h"
#include "clutseg/paramsel.h"

using namespace clutseg;
using namespace std;

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

// TODO: move method to Experiment
Experiment clone_setup(sqlite3* & db, int id) {
    Experiment e;
    e.id = id;
    e.deserialize(db);
    e.detach();
    e.response = Response();
    e.has_run = false;
    e.flags = 0;
    e.skip = false;
    e.vcs_commit = "";
    e.time = "";
    e.human_note = "";
    e.machine_note = "";
    return e;
}

// IDEA: We could use a branch-and-bound technique (skip) for stopping an
// experiment after we find (checking a couple of pictures) that it will
// definitely have lower success rate than others. The experiment must be set
// to 'skip' state since we cannot compute any result on it. Also a comment or
// some state flag must be set in order to make this decision transparent to
// the user. Finally, we must select the criteria we are interested in. Also,
// we should be careful, because if we skip experiments using FAST feature
// detector because ORB performed faster, we might lose valuable information.
// Also, in order to analyze WHY some configuration was bad, it might be better
// to run the experiment upon completion. Also, if the best experiment achieved
// 60% success rate, we still have to evaluate at least 9 pictures for deciding
// whether to stop the experiment. Another idea would be to take a very
// aggressive approach. We set the lower bound to maybe 75% and if we see more
// than 6 test scenes failing, we abandon this experiment. This will give a
// speed-up of factor 3.5. For ORB, this might be very much possible since I
// know that it's possible to achieve 66%.

void insert_experiments(sqlite3* & db) {
    // FAST + rBRIEF + LSH-BINARY
    {
        // This configuration works quite well compared to others, achieving
        // success rates up to 67% on the ias_kinect_test_grounded_21 test
        // dataset, getting classification right on up to 17/21 images. It is
        // also very fast in training and detection. Hence, this configuration
        // must be further investigated into, looking for further improvement
        // by exploring neighbored parameter space.
        Experiment e = createExperiment();
        e.name = "fast-rbrief-multiscale-lshbinary";
        insert_if_not_exist(db, e);
    }

    int i = 0;
    for (int detectMinInliersCount = 5; detectMinInliersCount <= 25; detectMinInliersCount += 5) { // 5
        for (int detectMaxProjectionError = 6; detectMaxProjectionError <= 18; detectMaxProjectionError += 3) { // 5
            for (int locateMinInliersCount = 5; locateMinInliersCount <= 30; locateMinInliersCount += 5) { // 6
                for (int locateMaxProjectionError = 6; locateMaxProjectionError <= 18; locateMaxProjectionError += 3) { // 5
                    // 5 * 6 * 5 * 5 = 30 * 25 = 750 
                    // needs approximately 12 hours to compute 
                    Experiment e = createExperiment();
                    e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % i);
                    e.paramset.detect_pms_guess.minInliersCount = detectMinInliersCount;
                    e.paramset.detect_pms_guess.maxProjectionError = detectMaxProjectionError;
                    e.paramset.locate_pms_guess.minInliersCount = locateMinInliersCount;
                    e.paramset.locate_pms_guess.maxProjectionError = locateMaxProjectionError;
                    insert_if_not_exist(db, e);
                    i++;
                }
            }
        }
    }

    sqlite3_stmt *select;
    string sql = "select experiment.id from experiment join response where experiment.response_id = response.id and experiment.id < 750 order by succ_rate desc limit 20";
    db_prepare(db, select, sql);
    cout << "[SQL] " << sql << endl;
    vector<int> ids;
    while (sqlite3_step(select) == SQLITE_ROW) {
        ids.push_back(sqlite3_column_int(select, 0));
    }
    sqlite3_finalize(select);

    // Test 20 best experiments according to success rate, use less RANSAC iterations to find out
    // whether we can save time here.
    assert(ids.size() == 20);
    size_t j = 750;
    for (size_t i = 0; i < ids.size(); i++, j++) {
        Experiment e = clone_setup(db, ids[i]);
        e.paramset.detect_pms_guess.ransacIterationsCount = 200;
        e.paramset.locate_pms_guess.ransacIterationsCount = 200;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-2";
	insert_if_not_exist(db, e);
    }

    // Test 20 best experiments according to success rate, and turn off ratio
    // test to see what happens.
    for (size_t i = 0; i < ids.size(); i++, j++) {
        Experiment e = clone_setup(db, ids[i]);
        e.paramset.detect_pms_match.doRatioTest = false;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-3";
	insert_if_not_exist(db, e);
    }

    // Test 20 best experiments according to success rate, and use smaller
    // threshold in recognition.
    for (size_t i = 0; i < ids.size(); i++, j++) {
        Experiment e = clone_setup(db, ids[i]);
        e.paramset.recog_pms_fe.detector_params["threshold"] = 15;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-4";
	insert_if_not_exist(db, e);
    }
 
    // Test 20 best experiments according to success rate, and use smaller
    // threshold in recognition and also smaller threshold in training, i.e. 
    // many more features.
    for (size_t i = 0; i < ids.size(); i++, j++) {
        Experiment e = clone_setup(db, ids[i]);
        e.paramset.train_pms_fe.detector_params["threshold"] = 20;
        e.paramset.recog_pms_fe.detector_params["threshold"] = 15;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-5";
	insert_if_not_exist(db, e);
    }
  
    // Test 20 best experiments according to success rate, and use smaller
    // threshold in recognition and also smaller threshold in training, i.e. 
    // many many more features, but also less RANSAC iterations
    for (size_t i = 0; i < ids.size(); i++, j++) {
        Experiment e = clone_setup(db, ids[i]);
        e.paramset.train_pms_fe.detector_params["threshold"] = 15;
        e.paramset.recog_pms_fe.detector_params["threshold"] = 10;
        e.paramset.detect_pms_guess.ransacIterationsCount = 200;
        e.paramset.locate_pms_guess.ransacIterationsCount = 200;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-6";
	insert_if_not_exist(db, e);
    }

    sqlite3_stmt *select100;
    sql = "select experiment.id from experiment join response where experiment.response_id = response.id and experiment.id < 750 order by succ_rate desc limit 100";
    db_prepare(db, select100, sql);
    cout << "[SQL] " << sql << endl;
    vector<int> ids100;
    while (sqlite3_step(select100) == SQLITE_ROW) {
        ids100.push_back(sqlite3_column_int(select100, 0));
    }
    sqlite3_finalize(select100);
    // Test 100 best experiments according to success rate, and use smaller
    // threshold in recognition and also smaller threshold in training, i.e. 
    // many many more features, but also more RANSAC iterations than in run-6
    for (size_t i = 0; i < ids100.size(); i++, j++) {
        Experiment e = clone_setup(db, ids100[i]);
        e.paramset.train_pms_fe.detector_params["threshold"] = 15;
        e.paramset.recog_pms_fe.detector_params["threshold"] = 10;
        e.paramset.detect_pms_guess.ransacIterationsCount = 750;
        e.paramset.locate_pms_guess.ransacIterationsCount = 750;
        e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
        e.batch = "run-7";
        insert_if_not_exist(db, e);
    }

    // Test 100 best experiments according to success rate, and use smaller
    // threshold in recognition and also smaller threshold in training, i.e. 
    // as many features as in run-7, but even more RANSAC iterations than in run-7
    // and smaller max_projection_error in locating step than in run-7
    for (size_t i = 0; i < ids100.size(); i++) {
	    for (size_t locateMaxProjectionError = 6; locateMaxProjectionError <= 15 ; locateMaxProjectionError += 3, j++) {
         	Experiment e = clone_setup(db, ids100[i]);
            e.paramset.train_pms_fe.detector_params["threshold"] = 15;
            e.paramset.recog_pms_fe.detector_params["threshold"] = 10;
            e.paramset.locate_pms_guess.maxProjectionError = locateMaxProjectionError;
            e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
            e.batch = "run-8";
            insert_if_not_exist(db, e);
        }
    }

    sqlite3_stmt *select100all;
    sql = "select experiment.id from experiment join response where experiment.response_id = response.id and experiment.id order by succ_rate desc limit 100";
    db_prepare(db, select100all, sql);
    cout << "[SQL] " << sql << endl;
    vector<int> ids100all;
    while (sqlite3_step(select100all) == SQLITE_ROW) {
        ids100all.push_back(sqlite3_column_int(select100all, 0));
    }
    sqlite3_finalize(select100all);

    j = 1350;
	
    for (size_t i = 0; i < ids100all.size(); i++) {
        for (int detectKnn = 3; detectKnn <= 5; detectKnn++) {
            for (int locateKnn = 3; locateKnn <= 5; locateKnn++, j++) {
                Experiment e = clone_setup(db, ids100all[i]);
                e.paramset.detect_pms_match.knn = detectKnn;
                e.paramset.locate_pms_match.knn = locateKnn;
                e.name = str(boost::format("fast-rbrief-multiscale-lshbinary-%d") % j);
                e.batch = "run-9";
                insert_if_not_exist(db, e);
            }
        }
    }

    /* that one is nonsense
    // SIFT + rBRIEF + LSH-BINARY (extractor_type=multi-scale)
    {
        // This one is a full-blown failure, not producing anything to start
        // with.  Either there is an important parameter set to a bad value or
        // there is still some bug somewhere, probably already during training.
        // NOTE: I forgot to change the extractor_type to 'sequential' here.
        // Leave it for comparison.
        Experiment e = createExperiment();
        e.name = "sift-rbrief-multiscale-lshbinary";
        e.paramset.train_pms_fe.detector_type = "SIFT";
        e.paramset.recog_pms_fe.detector_type = "SIFT";
        insert_if_not_exist(db, e);
    } */

    /* that one is nonsense
    // SURF + rBRIEF + LSH-BINARY (extractor_type=multi-scale)
    {
        // This one is a full-blown failure, not producing anything to start
        // with.  Either there is an important parameter set to a bad value or
        // there is still some bug somewhere, probably already during training.
        // NOTE: I forgot to change the extractor_type to 'sequential' here.
        // Leave it for comparison.
        Experiment e = createExperiment();
        e.name = "surf-rbrief-multiscale-lshbinary";
        e.paramset.train_pms_fe.detector_type = "SURF";
        e.paramset.recog_pms_fe.detector_type = "SURF";
        insert_if_not_exist(db, e);
    }*/

    /*
    // SIFT + rBRIEF + LSH-BINARY (extractor_type=sequential)
    {
        Experiment e = createExperiment();
        e.name = "sift-rbrief-sequential-lshbinary";
        e.paramset.train_pms_fe.detector_type = "SIFT";
        e.paramset.recog_pms_fe.detector_type = "SIFT";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        insert_if_not_exist(db, e);
    }

    // SIFT + rBRIEF + FLANN (extractor_type=sequential)
    {
        // This does not work. I think FLANN cannot be used
        // with binary feature descriptors. It also fails with
        // an error.
        Experiment e = createExperiment();
        e.name = "sift-rbrief-sequential-flann";
        e.paramset.train_pms_fe.detector_type = "SIFT";
        e.paramset.recog_pms_fe.detector_type = "SIFT";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        e.paramset.detect_pms_match.type = "FLANN";
        e.paramset.locate_pms_match.type = "FLANN";
        e.skip = true;
        e.human_note = "fails in FLANN";
        insert_if_not_exist(db, e);
    }

    // SIFT + FLANN (extractor_type=sequential)
    {
        Experiment e = createExperiment();
        e.name = "sift-sequential-flann";
        e.paramset.train_pms_fe.detector_type = "SIFT";
        e.paramset.recog_pms_fe.detector_type = "SIFT";
        e.paramset.train_pms_fe.descriptor_type = "SIFT";
        e.paramset.recog_pms_fe.descriptor_type = "SIFT";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        e.paramset.detect_pms_match.type = "FLANN";
        e.paramset.locate_pms_match.type = "FLANN";
        insert_if_not_exist(db, e);
    }

    // SURF + rBRIEF + LSH-BINARY (extractor_type=sequential)
    {
        Experiment e = createExperiment();
        e.name = "surf-rbrief-sequential-lshbinary";
        e.paramset.train_pms_fe.detector_type = "SURF";
        e.paramset.recog_pms_fe.detector_type = "SURF";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        insert_if_not_exist(db, e);
    }

    // SURF + rBRIEF + FLANN (extractor_type=sequential)
    {
        Experiment e = createExperiment();
        e.name = "surf-rbrief-sequential-flann";
        e.paramset.train_pms_fe.detector_type = "SURF";
        e.paramset.recog_pms_fe.detector_type = "SURF";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        e.paramset.detect_pms_match.type = "FLANN";
        e.paramset.locate_pms_match.type = "FLANN";
        insert_if_not_exist(db, e);
    }

    // SURF + FLANN (extractor_type=sequential)
    {
        Experiment e = createExperiment();
        e.name = "surf-sequential-flann";
        e.paramset.train_pms_fe.detector_type = "SURF";
        e.paramset.recog_pms_fe.detector_type = "SURF";
        e.paramset.train_pms_fe.descriptor_type = "SURF";
        e.paramset.recog_pms_fe.descriptor_type = "SURF";
        e.paramset.train_pms_fe.extractor_type = "sequential";
        e.paramset.recog_pms_fe.extractor_type = "sequential";
        e.paramset.detect_pms_match.type = "FLANN";
        e.paramset.locate_pms_match.type = "FLANN";
        insert_if_not_exist(db, e);
    }

    // STAR + rBRIEF + LSH-BINARY
    {
        // Works only one one or two pictures. Seems to be an issue with
        // thresholds and other parameters, but should work in principle. 
        Experiment e = createExperiment();
        e.name = "star-rbrief-multiscale-lshbinary";
        e.paramset.train_pms_fe.detector_type = "STAR";
        e.paramset.recog_pms_fe.detector_type = "STAR";
        insert_if_not_exist(db, e);
    }*/

    /*
    // STAR + rBRIEF + LSH-BINARY
    {
        Experiment e = createExperiment();
        e.name = "star-rbrief-multiscale-lshbinary";
        e.paramset.train_pms_fe.detector_type = "STAR";
        e.paramset.recog_pms_fe.detector_type = "STAR";
        insert_if_not_exist(db, e);
    }

    // ORB + LSH-BINARY
    {
        // Achieves success rates up to 50%. Seems to be a prospective
        // candidate for further investigation.
        Experiment e = createExperiment();
        e.name = "orb-lshbinary";
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

    if (term) return;

    // ORB + LSH-BINARY + gridded (AMMSFast)
    {
        Experiment e = createExperiment();
        e.name = "orb-gridded-lshbinary";
        e.paramset.train_pms_fe.detector_type = "gridded";
        e.paramset.train_pms_fe.extractor_type = "ORB";
        e.paramset.train_pms_fe.descriptor_type = "ORB";
        e.paramset.train_pms_fe.detector_params["threshold"] = 0.000001;
        e.paramset.recog_pms_fe.detector_type = "gridded";
        e.paramset.recog_pms_fe.extractor_type = "ORB";
        e.paramset.recog_pms_fe.descriptor_type = "ORB";
        e.paramset.recog_pms_fe.detector_params["threshold"] = 0.000001;
        insert_if_not_exist(db, e);
    }

    if (term) return;

    // ORB + LSH-BINARY + low-threshold
    {
        // Wasn't bad, but not as good as with the standard threshold.
        Experiment e = createExperiment();
        e.name = "orb-lowthreshold-lshbinary";
        e.paramset.train_pms_fe.detector_type = "ORB";
        e.paramset.train_pms_fe.extractor_type = "ORB";
        e.paramset.train_pms_fe.descriptor_type = "ORB";
        e.paramset.train_pms_fe.detector_params["threshold"] = 0.0000001;
        e.paramset.recog_pms_fe.detector_type = "ORB";
        e.paramset.recog_pms_fe.extractor_type = "ORB";
        e.paramset.recog_pms_fe.descriptor_type = "ORB";
        e.paramset.recog_pms_fe.detector_params["threshold"] = 0.0000001;
        insert_if_not_exist(db, e);
    } */
}


int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Usage: experiment_enqueue <database>" << endl;
        return -1;
    }

    bfs::path db_path = argv[1];

    sqlite3* db;
    cout << "Opening database ..." << endl;
    db_open(db, db_path);

    cout << "Inserting experiment setups ..." << endl;
    insert_experiments(db);

    return 0;
}
