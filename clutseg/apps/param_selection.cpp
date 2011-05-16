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

#include "clutseg/db.h"
#include "clutseg/runner.h"

#include <iostream>
#include <vector>


using namespace clutseg;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
    // ... in order to convert it to a ROS node
    // #include <ros/ros.h>
    // #include <ros/console.h>
    // ros::init(argc, argv, "param_selection");
    // ros::NodeHandle n;

    if (argc != 3) {
        cerr << "Usage: param_selection <database> <train_cache>" << endl;
        return -1;
    }
    bfs::path db_path = argv[1];
    bfs::path cache_dir = argv[2];
    sqlite3* db;
    cout << "Opening database ..." << endl;
    db_open(db, db_path);
    TrainFeaturesCache cache(cache_dir);
    cout << "Running experiments ..." << endl;
    ExperimentRunner runner(db, cache);
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
            
    // IDEA: save git commit
    // take first word out of git log -1 --pretty=oneline
}

