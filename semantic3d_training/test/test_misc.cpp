/*
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <boost/lexical_cast.hpp>
#include "misc.h"

/** Extract an angle from a filename */
TEST(Misc, ExtractAngleFromFileName) {
    for (int a = -180; a <= 180; a += 30) {
        EXPECT_EQ(a, extractAngleFromFileName("bean-can_" + boost::lexical_cast<string>(a) + "_.log.delimited.rotated.pcd"));
    }
}

