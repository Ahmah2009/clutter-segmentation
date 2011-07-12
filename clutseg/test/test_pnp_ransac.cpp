/**
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>

#include <ctime>
#include <cv.h>
#include <iostream>

using namespace cv;
using namespace std;

class test_pnp_ransac : public ::testing::Test {
    public:
        virtual void SetUp() {
        }
};

/** Check whether the random number generator from OpenCV
 * requires manual initialization of the seed in order to
 * make it appear non-deterministic. */
TEST_F(test_pnp_ransac, rng_deterministic) {
    RNG rng;
    RNG rng2;
    cout << rng() << endl;
    cout << rng2() << endl;
    EXPECT_EQ(rng(), rng2());
    EXPECT_EQ(rng(), rng2());
    EXPECT_EQ(rng(), rng2());
}

/** Check whether initializing the seed gives the desired
 * pseudo-non-deterministic behaviour. */
TEST_F(test_pnp_ransac, rng_initialized_seed) {
    RNG rng;
    RNG rng2;
    rng2.state = time(NULL);
    cout << rng() << endl;
    cout << rng2() << endl;
    EXPECT_NE(rng(), rng2());
    EXPECT_NE(rng(), rng2());
    EXPECT_NE(rng(), rng2());
}

