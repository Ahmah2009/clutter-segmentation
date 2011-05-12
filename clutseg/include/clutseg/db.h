/**
 * Author: Julius Adorf
 */

#include <sqlite3.h>

#include <string>

namespace clutseg {

    void db_open(sqlite3 *db, const std::string & path);
    
    void db_exec(sqlite3 *db, std::string sql);

    void db_close(sqlite3 *db);

}

