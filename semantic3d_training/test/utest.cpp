/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include "misc.h"

using namespace std;

/** List all filenames in the current directory */
TEST(Misc, ListFiles)
{
    using namespace boost::filesystem;
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
TEST(Misc, ListFiles2)
{
    using namespace boost::filesystem;
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

/** Extract an angle from a filename */
TEST(Misc, ExtractAngleFromFileName) {
    for (int a = -180; a <= 180; a += 30) {
        EXPECT_EQ(a, extractAngleFromFileName("bean-can_" + boost::lexical_cast<std::string>(a) + "_.log.delimited.rotated.pcd"));
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

