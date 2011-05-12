/**
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <iostream>
#include <sqlite3.h>
#include <string>
 
using namespace std;

// TODO: move to clutseg_test
 
TEST(SqliteTest, OpenDB) {
    sqlite3* db;
    string Path = "build/OpenDBtest.sqlite3";
    if(sqlite3_open(Path.c_str(), &db) != SQLITE_OK)
    {
      cerr << sqlite3_errmsg(db) << endl;
    }
    sqlite3_close(db);
}
 
TEST(SqliteTest, CRUD) {
    sqlite3* db;
    sqlite3_open("build/test.sqlite3", &db);
    {
        /* insert */
        sqlite3_stmt* insert;
        sqlite3_prepare_v2(db, "INSERT INTO Persons VALUES (?, ?, ?, ?, ?)", 1024, &insert, NULL);
        sqlite3_bind_int(insert, 1, 42);
        sqlite3_bind_text(insert, 2, "Chan", 255, NULL);
        sqlite3_bind_text(insert, 3, "Jackie", 255, NULL);
        sqlite3_bind_text(insert, 4, "Kowloon Park Drive 2, 86F", 255, NULL);
        sqlite3_bind_text(insert, 5, "Hong Kong", 255, NULL);
        sqlite3_step(insert);
        sqlite3_finalize(insert);
    }

    {    
        /* read */
        sqlite3_stmt* read;
        sqlite3_prepare_v2(db, "SELECT * FROM Persons WHERE P_Id=?", 1024, &read, NULL);
        sqlite3_bind_int(read, 1, 42);
        sqlite3_step(read);
        sqlite3_finalize(read);
    }

    {
        /* update */
        sqlite3_stmt* update;
        sqlite3_prepare_v2(db, "UPDATE Persons SET FirstName=? WHERE P_Id=?", 1024, &update, NULL);
        sqlite3_bind_text(update, 1, "Andrew", 255, NULL);
        sqlite3_bind_int(update, 2, 42);
        sqlite3_step(update);
        sqlite3_finalize(update);
    }

    {
        /* delete */
        sqlite3_stmt* del;
        sqlite3_prepare_v2(db, "DELETE FROM Persons WHERE P_Id=?", 1024, &del, NULL);
        sqlite3_bind_int(del, 1, 42);
        sqlite3_step(del);
        sqlite3_finalize(del);
    }

    sqlite3_close(db);
}

TEST(SqliteTest, CreateTable) {
    sqlite3* db;
    sqlite3_open("build/test.sqlite3", &db);
    
    {
        // credits to http://www.w3schools.com/sql/sql_create_table.asp
        sqlite3_stmt* create;
        sqlite3_prepare_v2(db, "CREATE TABLE Persons2 "
                "("
                "    P_Id int,"
                "    LastName varchar(255),"
                "    FirstName varchar(255),"
                "    Address varchar(255),"
                "    City varchar(255)"
                ")", 1024, &create, NULL);
        sqlite3_step(create);   
        sqlite3_finalize(create);
    }

    sqlite3_close(db);
}
