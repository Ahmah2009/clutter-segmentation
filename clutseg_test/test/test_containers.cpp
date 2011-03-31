/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>
#include <map>

using namespace std;

TEST(Containers, UseCharToIntMap) {
    map<char, int> m;
    m['a'] = 42;
    EXPECT_EQ(42, m['a']);
}

TEST(Containers, DefineCharPointerToIntMap) {
    map<char*, int> m;
}

TEST(Containers, UseCharPointerToIntMap) {
    map<char*, int> m;
    char *foo = "hitchhiker";
    m[foo] = 42;
    EXPECT_EQ(42, m[foo]);
}

TEST(Containers, UseStringToIntMap) {
    map<string, int> m;
    m["hitchhiker"] = 42;
    EXPECT_EQ(42, m["hitchhiker"]);
}

TEST(Containers, UseCharPointerAndStringInterchangeablyAsKeys) {
    map<string, int> m;
    const char *foo = "hitchhiker";
    string foo2 = "hitchhiker";
    m[string(foo)] = 42;
    EXPECT_EQ(42, m[string(foo)]);
    EXPECT_EQ(42, m[foo2]);
}


