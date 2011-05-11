/**
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <iostream>
#include <sqlite3.h>
#include <string>
 
using namespace std;
 
TEST(SqliteTest, OpenDB) {
    sqlite3* Database;
    string Path = "build/deleteme.sqlite3";
    if(sqlite3_open(Path.c_str(), &Database) != SQLITE_OK)
    {
      cerr << sqlite3_errmsg(Database) << endl;
    }
    sqlite3_close(Database);
}

