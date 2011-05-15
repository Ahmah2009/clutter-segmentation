/**
 * Author: Julius Adorf 
 */

#include "clutseg/runner.h"

#include <gtest/gtest.h>

using namespace clutseg;
using namespace std;

struct RunnerTest : public ::testing::Test {

    void SetUp() {
        TrainFeaturesCache cache("build/train_cache");
        runner = ExperimentRunner(db, cache);
    }

    sqlite3* db;
    ExperimentRunner runner;

};

