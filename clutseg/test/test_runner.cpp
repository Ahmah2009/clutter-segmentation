/**
 * Author: Julius Adorf 
 */

#include "clutseg/runner.h"

#include <gtest/gtest.h>

using namespace clutseg;
using namespace std;

struct test_runner : public ::testing::Test {

    void SetUp() {
        ModelbaseCache cache("build/train_cache");
        ResultStorage storage("build/results");
        runner = ExperimentRunner(db, cache, storage);
    }

    sqlite3* db;
    ExperimentRunner runner;

};

TEST_F(test_runner, cloud_name) {
    map<string, string> names;
    names["image_00023.png"] = "cloud_00023.pcd";
    names["image_00023.jpg"] = "cloud_00023.pcd";
    names["directory/image_00023.png"] = "directory/cloud_00023.pcd";
    names["dir_A/dir_B.png/image_00023.png"] = "dir_A/dir_B.png/cloud_00023.pcd";
    names["img_00023.png"] = "cloud_00023.pcd";
    names["image_23.jpg"] = "cloud_23.pcd";
    names["image.jpg"] = "image.jpg.cloud.pcd";
    map<string, string>::iterator it = names.begin();
    map<string, string>::iterator end = names.end();
    while (it != end) {
        EXPECT_EQ(it->second, cloudPath(it->first).string());
        it++;
    }
}

TEST_F(test_runner, print_num_procs_online) {
    cout << sysconf(_SC_NPROCESSORS_ONLN) << endl;
}

