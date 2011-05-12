/**
 * Author: Julius Adorf
 */

#include <clutseg/db.h>

#include <boost/format.hpp>
#include <gtest/gtest.h>
#include <iostream>

using namespace clutseg;
using namespace std;

struct DbTest : public ::testing::Test {

    void SetUp() {
        sqlite3_open("build/db.sqlite3", &db);
    }

    void TearDown() {
        sqlite3_close(db);
    }

    sqlite3 *db;
    
};

TEST_F(DbTest, OpenNonExisting) {
    sqlite3 *ldb = NULL;
    db_open(ldb, str(boost::format("build/db%s.sqlite3") % rand()));
}

TEST_F(DbTest, Close) {
    db_close(db);
}

TEST_F(DbTest, ExecGood) {
    db_exec(db, "drop table good;"); 
    db_exec(db, "create table good (id integer primary key);"); 
}

TEST_F(DbTest, ExecBad) {
    try {
        db_exec(db, "create TABBLE foobar (id integer primary key);"); 
    } catch (ios_base::failure f) {
        EXPECT_TRUE(string(f.what()).find("Error") != string::npos);
    }
}

