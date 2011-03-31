/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>

using namespace std;
using namespace boost;

TEST(ProgramOptions, ReadConfigFile) {
    fstream f;
    f.open("./data/sample.ini");
    program_options::options_description desc;
    string var;
    desc.add_options()("section.foo", program_options::value<string>(&var));
    program_options::parse_config_file(f, desc);
}

