/**
 * Author: Julius Adorf
 */

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <sqlite3.h>
#include <string>
 
using namespace std;

TEST(SqliteTest, CRUD) {
    sqlite3* db;
    string fn = "build/SqliteTest.CRUD.sqlite3";
    boost::filesystem::remove(fn);
    EXPECT_EQ(SQLITE_OK, sqlite3_open(fn.c_str(), &db));

    {
        sqlite3_stmt* table;
        EXPECT_EQ(SQLITE_OK, sqlite3_prepare_v2(db,
            "CREATE TABLE simple ("
            "   id INTEGER PRIMARY KEY,"
            "   text TEXT,"
            "   fpnum REAL,"
            "   intnum INT"
            ");",
        -1, &table, NULL));
        EXPECT_EQ(SQLITE_DONE, sqlite3_step(table));
        EXPECT_EQ(SQLITE_OK, sqlite3_finalize(table));
    }

    {
        /* insert */
        sqlite3_stmt* insert;
        EXPECT_EQ(SQLITE_OK,  sqlite3_prepare_v2(db, "INSERT INTO simple VALUES (null, 'sometext', 1.5, 2)", -1, &insert, NULL));
        EXPECT_EQ(SQLITE_DONE, sqlite3_step(insert));
        EXPECT_EQ(SQLITE_OK, sqlite3_finalize(insert));
    }

    {    
        /* read */
        sqlite3_stmt* read;
        EXPECT_EQ(SQLITE_OK, sqlite3_prepare_v2(db, "SELECT text, fpnum, intnum FROM simple WHERE id=1;", -1, &read, NULL));
        EXPECT_EQ(SQLITE_ROW, sqlite3_step(read));
        EXPECT_EQ("sometext", string((const char*) (sqlite3_column_text(read, 0))));
        EXPECT_EQ(1.5, sqlite3_column_double(read, 1));
        EXPECT_EQ(2, sqlite3_column_int(read, 2));
        EXPECT_EQ(SQLITE_DONE, sqlite3_step(read));
        sqlite3_finalize(read);
    }

    {
        /* update */
        sqlite3_stmt* update;
        EXPECT_EQ(SQLITE_OK, sqlite3_prepare_v2(db, "UPDATE simple SET text=? WHERE id=?", -1, &update, NULL));
        // Be careful, indexing into columns when reading starts with zero, but
        // indices for binding arguments start with one!
        EXPECT_EQ(SQLITE_OK, sqlite3_bind_text(update, 1, "someothertext", -1, NULL));
        EXPECT_EQ(SQLITE_OK, sqlite3_bind_int(update, 2, 1));
        EXPECT_EQ(SQLITE_DONE, sqlite3_step(update));
        EXPECT_EQ(SQLITE_OK, sqlite3_finalize(update));
    }

    {
        /* delete */
        sqlite3_stmt* del;
        EXPECT_EQ(SQLITE_OK, sqlite3_prepare_v2(db, "DELETE FROM simple WHERE id=?", 1024, &del, NULL));
        EXPECT_EQ(SQLITE_OK, sqlite3_bind_int(del, 1, 42));
        EXPECT_EQ(SQLITE_DONE, sqlite3_step(del));
        EXPECT_EQ(SQLITE_OK, sqlite3_finalize(del));
    }

    EXPECT_EQ(SQLITE_OK, sqlite3_close(db));
}

