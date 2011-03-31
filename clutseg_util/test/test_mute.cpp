/**
 * Author: Julius Adorf
 */

#include "test.h"
#include "mute.h"

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

using namespace std;

TEST(Mute, MuteCout) {
    fstream nullstream;
    nullstream.open("/dev/null", ios_base::out);
    streambuf * nullbuf = nullstream.rdbuf();
    Mute m(nullbuf);   
    m.disable();
    cout << "ERROR!" << endl;
    m.enable();
    cout << "OK!" << endl;
    ASSERT_TRUE(true);
}

