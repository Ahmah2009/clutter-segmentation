/*
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"

#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>

using namespace std;
using namespace clutseg;

TEST(GroundTest, LoadTestSetGroundTruthWithoutPoses) {
    TestSetGroundTruth m = loadTestSetGroundTruthWithoutPoses("./data/testdesc.txt");
}

TEST(GroundTest, ReadTestSetGroundTruthWithoutPoses) {
    TestSetGroundTruth m = loadTestSetGroundTruthWithoutPoses("./data/testdesc.txt");
    GroundTruth s = m["t0000.png"];

    bool true_pos = false;
    BOOST_FOREACH(const NamedPose & np, s) {
        if (np.name == "downy") {
            true_pos = true;
        }
        ASSERT_FALSE(np.pose.estimated);
    }
    ASSERT_FALSE(true_pos);
}

