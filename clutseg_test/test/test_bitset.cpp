/**
 * Author: Julius Adorf
 */

#include "test.h"

#include <boost/dynamic_bitset.hpp>

TEST(Bitset, Create66Bitset) {
    size_t size = 66;
    boost::dynamic_bitset<> bs(size);
    bs.set(0);
}

