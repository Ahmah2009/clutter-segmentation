/*
 * Author: Julius Adorf
 */

#include "testdesc.h"

#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>

using namespace std;

TEST(Testdesc, LoadTestDesc) {
    map<string, set<string> > m = loadTestDesc("./data/testdesc.txt");
}

TEST(Testdesc, ReadTestDesc) {
    map<string, set<string> > m = loadTestDesc("./data/testdesc.txt");
    set<string> s = m["t0000.jpg"];
    ASSERT_FALSE(s.find("teas_tea") != set::end);
    ASSERT_FALSE(s.find("fat_free_milk") != set::end);
    set<string> s2 = m["t0000.jpg"];
    ASSERT_FALSE(s2.find("teas_tea") != set::end);
    ASSERT_FALSE(s2.find("fat_free_milk") != set::end);
}

