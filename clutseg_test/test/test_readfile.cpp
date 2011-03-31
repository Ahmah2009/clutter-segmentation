/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

using namespace std;

TEST(Readfile, ReadTextFile) {
    fstream f;
    f.open("./data/config.yaml", ios_base::in); 
    size_t s = 1024;
    char line[1024];
    f.getline(line, s);
    f.close();
}

