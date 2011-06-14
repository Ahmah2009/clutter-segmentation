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

TEST_F(test_flags, set) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, set_twice) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, clear) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, clear_twice) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, set_clear_set) {
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
    flag.clear();
    EXPECT_FALSE(flag.exists());
    flag.set();
    EXPECT_TRUE(flag.exists());
}

TEST_F(test_flags, dir_not_a_flag) {
    EXPECT_FALSE(flag.exists());
    bfs::create_directory(flagp);
    EXPECT_FALSE(flag.exists());
}

TEST_F(test_flags, set_flag_fails_if_dir_with_same_name_exists) {
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

TEST_F(test_flags, clear_flag_fails_if_dir_with_same_name_exists) {
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

