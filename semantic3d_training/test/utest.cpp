/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
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
    vector<string> fn;
    fn.push_back("bean-can_-180_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_-150_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_-120_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_-90_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_-60_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_-30_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_0_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_30_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_60_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_90_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_120_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_150_.log.delimited.rotated.pcd");
    fn.push_back("bean-can_180_.log.delimited.rotated.pcd");
    EXPECT_EQ(13, fn.size());
    for (int i = 0; i < fn.size(); i++) {
        EXPECT_EQ(-180 + i * 30, extractAngleFromFileName(fn[i]));
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

