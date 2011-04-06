/**
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <iostream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

using namespace std;
using namespace boost;

TEST(Random, DiscreteUniformSample) {
    mt19937 twister;
    uniform_int<> six(1,6);
    variate_generator<mt19937&, uniform_int<> > die(twister, six);
    die();
}

TEST(Random, NormalSample) {
    normal_distribution<> n;
    mt19937 twister; 
    variate_generator<mt19937&, normal_distribution<> > normal_sample(twister, n);
    for (int i = 0; i < 25; i++) {
        cout << normal_sample();
        cout << " ";
    }
    cout << endl;
}

