#include "test.h"
#include <iostream>
#include <cmath>
#include <boost/format.hpp>

using namespace std;

TEST(Misc, PrintDouble) {
    double d = 13.2352395252233;;
    cout << d << endl;
    cout << boost::format("%15.13f") % d << endl;
}
