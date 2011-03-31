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

TEST(Containers, IterateOverMap) {
    map<string, int> m;
    m["alpha"] = 1;
    m["beta"] = 2;
    m["gamma"] = 3;
    m["delta"] = 4;
    string str;
    int sum = 0;
    for (map<string, int>::iterator it = m.begin(), end = m.end(); it != end; it++) {
        str += it->first;
        sum += it->second;
    }
    EXPECT_TRUE(str.find("alpha") != string::npos);
    EXPECT_TRUE(str.find("beta") != string::npos);
    EXPECT_TRUE(str.find("gamma") != string::npos);
    EXPECT_TRUE(str.find("delta") != string::npos);
    EXPECT_EQ(10, sum);
}
