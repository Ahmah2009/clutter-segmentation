/*
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <iostream>
#include "misc.h"

using namespace std;

/** Extract an angle from a filename */
TEST(Misc, ExtractAngleFromFileName) {
    for (int a = -180; a <= 180; a += 30) {
        EXPECT_EQ(a, extractAngleFromFileName("bean-can_" + boost::lexical_cast<string>(a) + "_.log.delimited.rotated.pcd"));
    }
}

TEST(Misc, TestBoostFormat) {
    EXPECT_EQ("0001", str(boost::format("%04i") %  1));
    EXPECT_EQ("0030", str(boost::format("%04i") % 30));
    EXPECT_EQ("-060", str(boost::format("%04i") % -60 ));
    EXPECT_EQ("0000", str(boost::format("%04i") % 0));
}

