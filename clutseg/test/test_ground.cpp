/*
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"

#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>

using namespace std;
using namespace clutseg;

TEST(Testdesc, LoadTestSetGroundTruthWithoutPoses) {
    TestSetGroundTruth m = loadTestSetGroundTruthWithoutPoses("./data/testdesc.txt");
}

TEST(Testdesc, ReadTestSetGroundTruthWithoutPoses) {
    TestSetGroundTruth m = loadTestSetGroundTruthWithoutPoses("./data/testdesc.txt");
    vector<NamedPose> s = m["t0000.png"];

    bool true_pos = false;
    BOOST_FOREACH(const NamedPose & np, expected) {
        if (np.name == "teas_tea") {
            true_pos = true;
        }
    }
    ASSERT_FALSE(true_pos);
}

