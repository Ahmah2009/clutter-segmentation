/**
 * Author: Julius Adorf
 */

#include "clutseg/paramsel.h"
#include "clutseg/db.h"

#include <boost/format.hpp>
#include <iostream>

using namespace std;

namespace clutseg {

    int64_t Serializable::serialize(sqlite3* db) {
        throw ios_base::failure("Cannot serialize, not implemented.");
    }

    void Serializable::deserialize(sqlite3* db, int64_t id) {
        throw ios_base::failure("Cannot deserialize, not implemented.");
    }

    int64_t ClutsegParams::serialize(sqlite3* db) {
        db_exec(db, boost::format(
            "insert into pms_clutseg (accept_threshold, ranking) values (%f, '%s');"
        ) % accept_threshold % ranking);
        return sqlite3_last_insert_rowid(db);
    }

    void ClutsegParams::deserialize(sqlite3* db, int64_t id) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select accept_threshold, ranking from pms_clutseg where id=%d") % id);
        db_step(read, SQLITE_ROW);
        accept_threshold = sqlite3_column_double(read, 0);
        ranking = string((const char*) sqlite3_column_text(read, 1));
        sqlite3_finalize(read);
    }

    int64_t Paramset::serialize(sqlite3* db) {
        return -1;
    }

    void Paramset::deserialize(sqlite3* db, int64_t id) {
    }

    int64_t Response::serialize(sqlite3* db) {
        db_exec(db, boost::format(
            "insert into response (value) values (%f);"
        ) % value);
        return sqlite3_last_insert_rowid(db);
    }

    void Response::deserialize(sqlite3* db, int64_t id) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select value from response where id=%d") % id);
        db_step(read, SQLITE_ROW);
        value = sqlite3_column_double(read, 0);
        sqlite3_finalize(read);
    }

    int64_t Experiment::serialize(sqlite3* db) {
        return -1;
    }
    
    void Experiment::deserialize(sqlite3* db, int64_t id) {

    }

}

