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
    EXPECT_TRUE(isObjectExpected(s, "teas_tea"));
    EXPECT_TRUE(isObjectExpected(s, "fat_free_milk"));
    EXPECT_FALSE(isObjectExpected(s, "downy"));
    EXPECT_FALSE(isObjectExpected(s, ""));
    EXPECT_FALSE(isObjectExpected(s, " "));
}

