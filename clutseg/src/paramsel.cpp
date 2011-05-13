/**
 * Author: Julius Adorf
 */

#include "clutseg/paramsel.h"
#include "clutseg/db.h"

#include <boost/format.hpp>
#include <iostream>

using namespace std;

namespace clutseg {

    void Serializable::serialize(sqlite3* db) {
        throw ios_base::failure("Cannot serialize, not implemented.");
    }

    void Serializable::deserialize(sqlite3* db) {
        throw ios_base::failure("Cannot deserialize, not implemented.");
    }

    void ClutsegParams::serialize(sqlite3* db) {
        db_exec(db, boost::format(
            "insert into pms_clutseg (accept_threshold, ranking) values (%f, '%s');"
        ) % accept_threshold % ranking);
        id = sqlite3_last_insert_rowid(db);
    }

    void ClutsegParams::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select accept_threshold, ranking from pms_clutseg where id=%d") % id);
        db_step(read, SQLITE_ROW);
        accept_threshold = sqlite3_column_double(read, 0);
        ranking = string((const char*) sqlite3_column_text(read, 1));
        sqlite3_finalize(read);
    }

    void Paramset::serialize(sqlite3* db) {
    }

    void Paramset::deserialize(sqlite3* db) {
    }

    void Response::serialize(sqlite3* db) {
        db_exec(db, boost::format(
            "insert into response (value) values (%f);"
        ) % value);
        id = sqlite3_last_insert_rowid(db);
    }

    void Response::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select value from response where id=%d") % id);
        db_step(read, SQLITE_ROW);
        value = sqlite3_column_double(read, 0);
        sqlite3_finalize(read);
    }

    void Experiment::serialize(sqlite3* db) {
    }
    
    void Experiment::deserialize(sqlite3* db) {
    }

}

