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

TEST(GroundTest, LoadSetGroundTruthWithoutPoses) {
    SetGroundTruth m = loadSetGroundTruthWithoutPoses("./data/testdesc.txt");
}

TEST(GroundTest, ReadSetGroundTruthWithoutPoses) {
    SetGroundTruth m = loadSetGroundTruthWithoutPoses("./data/testdesc.txt");
    GroundTruth s = m["t0000.png"];

    BOOST_FOREACH(const NamedPose & np, s.labels) {
        EXPECT_FALSE(np.pose.estimated);
    }
    EXPECT_TRUE(s.onScene("teas_tea"));
    EXPECT_TRUE(s.onScene("fat_free_milk"));
    EXPECT_FALSE(s.onScene("downy"));
    EXPECT_FALSE(s.onScene(""));
    EXPECT_FALSE(s.onScene(" "));
}

