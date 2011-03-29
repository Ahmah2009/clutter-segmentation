/**
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

using namespace boost;

TEST(Random, DiscreteUniformSample) {
    mt19937 twister;
    uniform_int<> six(1,6);
    variate_generator<mt19937&, uniform_int<> > die(twister, six);
    int x = die();
}

TEST(Random, NormalSample) {
    normal_distribution<> n;
    mt19937 twister; 
    variate_generator<mt19937&, normal_distribution<> > normal_sample(twister, n);
    double x = normal_sample();
}

