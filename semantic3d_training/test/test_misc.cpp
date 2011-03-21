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

TEST(Misc, AngleToFourChars) {
    EXPECT_EQ("-180", angleToFourChars(-180)); 
    EXPECT_EQ("-150", angleToFourChars(-150)); 
    EXPECT_EQ("-120", angleToFourChars(-120)); 
    EXPECT_EQ("-090", angleToFourChars(-90)); 
    EXPECT_EQ("-060", angleToFourChars(-60)); 
    EXPECT_EQ("-030", angleToFourChars(-30)); 
    EXPECT_EQ("0000", angleToFourChars(0)); 
    EXPECT_EQ("0030", angleToFourChars(30)); 
    EXPECT_EQ("0060", angleToFourChars(60)); 
    EXPECT_EQ("0090", angleToFourChars(90)); 
    EXPECT_EQ("0120", angleToFourChars(120)); 
    EXPECT_EQ("0150", angleToFourChars(150)); 
    EXPECT_EQ("0180", angleToFourChars(180)); 
}

