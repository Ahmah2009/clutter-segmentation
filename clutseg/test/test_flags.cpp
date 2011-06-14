/**
 * Author: Julius Adorf
 */

#include "clutseg/flags.h"

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

using namespace clutseg;
namespace bfs = boost::filesystem;

struct test_flags : public ::testing::Test {

    void SetUp() {
        flagp = bfs::path("build/test_flags.flag");
        flag = FileFlag(flagp);
    }

    void TearDown() {
        bfs::remove(flagp);
    }

    bfs::path flagp;
    FileFlag flag;

};

TEST_F(test_flags, SetFlag) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, SetFlagTwice) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, ClearFlag) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, ClearFlagTwice) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, SetClearSetFlag) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, DirectoryIsNotAFlag) {
    EXPECT_FALSE(flag.exists());
    bfs::create_directory(flagp);
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, SetFlagFailsIfDirectoryWithSameNameExists) {
    EXPECT_FALSE(flag.exists());
    bfs::create_directory(flagp);
    EXPECT_TRUE(bfs::is_directory(flagp));
    EXPECT_FALSE(flag.exists());
    try {
        flag.set();
        EXPECT_TRUE(false);
    } catch ( ... ) { }
    EXPECT_FALSE(flag.exists());
    EXPECT_TRUE(bfs::is_directory(flagp));
    bfs::remove(flagp);
}

TEST_F(test_flags, ClearFlagFailsIfDirectoryWithSameNameExists) {
    EXPECT_FALSE(flag.exists());
    bfs::create_directory(flagp);
    EXPECT_TRUE(bfs::is_directory(flagp));
    EXPECT_FALSE(flag.exists());
    try {
        flag.clear();
        EXPECT_TRUE(false);
    } catch ( ... ) { }
    EXPECT_FALSE(flag.exists());
    EXPECT_TRUE(bfs::is_directory(flagp));
}

