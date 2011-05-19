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

TEST(Testdesc, LoadTestSetGroundTruth) {
    TestSetGroundTruth m = loadTestSetGroundTruth("./data/testdesc.txt");
}

TEST(Testdesc, ReadTestSetGroundTruth) {
    TestSetGroundTruth m = loadTestSetGroundTruth("./data/testdesc.txt");
    set<string> s = m["t0000.png"];
    ASSERT_FALSE(s.find("teas_tea") == s.end());
    ASSERT_FALSE(s.find("fat_free_milk") == s.end());
}

