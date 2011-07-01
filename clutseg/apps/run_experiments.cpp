/**
 * Author: Julius Adorf
 *
 * This application shall find good parameters for tod_training, tod_detecting
 * and clutseg. Results are stored in a sqlite3 database.
 */

#include "clutseg/check.h"
#include "clutseg/db.h"
#include "clutseg/runner.h"
#include "clutseg/storage.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <csignal>
    #include <cstdio>
    #include <cstdlib>
    #include <iostream>
    #include <vector>
#include "clutseg/gcc_diagnostic_enable.h"


using namespace clutseg;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

ExperimentRunner runner;

bool term;

void terminate_hnd(int /* s */) {
    cout << "Received SIGINT" << endl;
    runner.terminate = true;
    term = true;
}

int main(int argc, char **argv) {
    // ... in order to convert it to a ROS node
    // #include <ros/ros.h>
    // #include <ros/console.h>
    // ros::init(argc, argv, "param_selection");
    // ros::NodeHandle n;

    if (argc != 4 && argc != 5) {
        cerr << "Usage: run_experiments <database> <train_cache> <result_dir>" << endl;
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
 
    term = false;

    // Use custom signal handler
    void (*prev_fn)(int);
    prev_fn = signal (SIGINT, terminate_hnd);
    if (prev_fn==SIG_IGN) signal (SIGINT,SIG_IGN);

    if (term) return 1;

    TrainFeaturesCache cache(cache_dir);
    ResultStorage storage(result_dir);
    cout << "Running experiments ..." << endl;
    runner = ExperimentRunner(db, cache, storage);

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

