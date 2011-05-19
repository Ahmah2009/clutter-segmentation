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

    BOOST_FOREACH(const NamedPose & np, s) {
        EXPECT_FALSE(np.pose.estimated);
    }
    EXPECT_TRUE(isObjectExpected("teas_tea"));
    EXPECT_TRUE(isObjectExpected("fat_free_milk"));
    EXPECT_FALSE(isObjectExpected("downy"));
    EXPECT_FALSE(isObjectExpected(""));
    EXPECT_FALSE(isObjectExpected(" "));
}

