/**
 * Author: Julius Adorf
 */

#include <clutseg/db.h>

#include <boost/format.hpp>
#include <iostream>
#include <string>
 
using namespace std;
using namespace boost;

namespace bfs = boost::filesystem;

namespace clutseg {

    void db_open(sqlite3 *&db, const bfs::path & path) {
        if (sqlite3_open(path.string().c_str(), &db) != SQLITE_OK) {
            throw ios_base::failure("Error when opening database: " + string(sqlite3_errmsg(db)));
        }
    }
    
    void db_exec(sqlite3 *&db, const string & sql) {
        // TODO: write to log file
        cout << "[SQL] " << sql << endl;
        char *errmsg;
        sqlite3_exec(db, sql.c_str(), NULL, NULL, &errmsg);
        if (errmsg != NULL) {
            string e(errmsg);
            sqlite3_free(errmsg);
            throw ios_base::failure("Error when executing SQL statement: " + e + "\nSQL: " + sql);
        }
        sqlite3_free(errmsg);
    }
 
    void db_exec(sqlite3* & db, boost::format & sql) {
        db_exec(db, sql.str()); 
    }
 
    void db_prepare(sqlite3* & db, sqlite3_stmt* & stmt, const std::string & sql) {
        if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, NULL) != SQLITE_OK) {
            throw ios_base::failure("Error when preparing statement: " + string(sqlite3_errmsg(db)));
        }
    }

    void db_prepare(sqlite3* & db, sqlite3_stmt* & stmt, const boost::format & sql) {
        db_prepare(db, stmt, sql.str());
    }

    void db_step(sqlite3_stmt* & stmt, int expected_status) {
        int a = sqlite3_step(stmt);
        // TODO: write to log-file
        cout << "[SQL] " << sqlite3_sql(stmt) << endl;
        if (a != expected_status) {
            throw ios_base::failure(str(boost::format(
                "Error when calling step: expected status %d, but was %d.\nSQL: %s") 
                    % expected_status % a % sqlite3_sql(stmt)));
        }
    }

    void db_close(sqlite3* & db) {
        if (sqlite3_close(db) != SQLITE_OK) {
            throw ios_base::failure("Error when closing database: " + string(sqlite3_errmsg(db)));
        }
    }

}

