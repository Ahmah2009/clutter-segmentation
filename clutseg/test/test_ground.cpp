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

    BOOST_FOREACH(const NamedPose & np, s.labels) {
        EXPECT_FALSE(np.pose.estimated);
    }
    EXPECT_TRUE(s.isObjectExpected("teas_tea"));
    EXPECT_TRUE(s.isObjectExpected("fat_free_milk"));
    EXPECT_FALSE(s.isObjectExpected("downy"));
    EXPECT_FALSE(s.isObjectExpected(""));
    EXPECT_FALSE(s.isObjectExpected(" "));
}

