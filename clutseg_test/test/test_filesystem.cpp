/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::filesystem;

/** List all filenames in the current directory */
TEST(Filesystem, ListFiles)
{
    bool has_test = false;
    bool has_manifest = false;
    for (directory_iterator it("."), end; it != end; ++it) {
        string fname(it->path().filename());
        cout << fname << endl;
        if (fname.find("test") != string::npos) {
            has_test = true;
        } else if (fname.find("manifest.xml") != string::npos) {
            has_manifest = true;
        }
    }
    EXPECT_TRUE(has_test);
    EXPECT_TRUE(has_manifest);
}

/** List all filenames in the current directory */
TEST(Filesystem, ListFiles2)
{
    vector<path> v;
    copy(directory_iterator("."), directory_iterator(), back_inserter(v));
    bool has_test = false;
    bool has_manifest = false;
    for (vector<path>::const_iterator it(v.begin()); it != v.end(); ++it) {
        string fname = it->filename();
        cout << fname << endl;
        if (fname.find("test") != string::npos) {
            has_test = true;
        } else if (fname.find("manifest.xml") != string::npos) {
            has_manifest = true;
        }
    }
    EXPECT_TRUE(has_test);
    EXPECT_TRUE(has_manifest);
}

