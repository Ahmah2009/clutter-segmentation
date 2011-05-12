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
        db_exec(db, str(boost::format(
            "insert into pms_clutseg (accept_threshold, ranking) values (%f, '%s');"
        ) % accept_threshold % ranking));
        return sqlite3_last_insert_rowid(db);
    }

    void ClutsegParams::deserialize(sqlite3* db, int64_t id) {
    }

    int64_t Paramset::serialize(sqlite3* db) {
        return -1;
    }

    void Paramset::deserialize(sqlite3* db, int64_t id) {

    }

    int64_t Response::serialize(sqlite3* db) {
        return -1;
    }

    void Response::deserialize(sqlite3* db, int64_t id) {

    }

    int64_t Experiment::serialize(sqlite3* db) {
        return -1;
    }
    
    void Experiment::deserialize(sqlite3* db, int64_t id) {

    }

}

