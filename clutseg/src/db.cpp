/**
 * Author: Julius Adorf
 */

#include <clutseg/db.h>

#include <iostream>
#include <string>

using namespace std;

namespace clutseg {

    void db_open(sqlite3 *db, const string & path) {
        if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) {
            throw ios_base::failure("Error when opening database: " + string(sqlite3_errmsg(db)));
        }
    }
    
    void db_exec(sqlite3 *db, string sql) {
        char *errmsg;
        sqlite3_exec(db, sql.c_str(), NULL, NULL, &errmsg);
        if (errmsg != NULL) {
            throw ios_base::failure("Error when executing SQL statement: " + string(errmsg));
        }
    }

    void db_close(sqlite3 *db) {
        if (sqlite3_close(db) != SQLITE_OK) {
            throw ios_base::failure("Error when closing database: " + string(sqlite3_errmsg(db)));
        }
    }

}

