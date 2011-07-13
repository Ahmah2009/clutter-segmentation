/*
 * Author: Julius Adorf
 */

#ifndef _DB_H_
#define _DB_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <boost/format/format_class.hpp>
    #include <sqlite3.h>
    #include <string>
#include "clutseg/gcc_diagnostic_enable.h"

/**
 * Tiny abstraction layer around sqlite3. Especially, shall use exceptions
 * instead of error codes, hiding some parameters and avoiding use of 'const
 * char *' and 'const unsigned char *'.
 */
namespace clutseg {
    
    /** \brief Opens a SQLite database.
     *
     * Delegates to sqlite3_open. Throws ios_base::failure if an error occurs.
     */
    void db_open(sqlite3* & db, const boost::filesystem::path & filename);
    
    /** \brief Executes a SQL statement.
     *
     * Delegates to sqlite3_exec. Throws ios_base::failure if an error occurs.
     */ 
    void db_exec(sqlite3* & db, const std::string & sql);

    /** \brief Executes a SQL statement.
     *
     * Delegates to sqlite3_exec. Throws ios_base::failure if an error occurs.
     */
    void db_exec(sqlite3* & db, boost::format & sql);
   
    /** \brief Prepares a SQL statement.
     *
     * Delegates to sqlite3_prepare_v2. Throws ios_base::failure if an error occurs.
     */
    void db_prepare(sqlite3* & db, sqlite3_stmt* & stmt, const std::string & sql);

    /** \brief Prepares a SQL statement.
     *
     * Delegates to sqlite3_prepare_v2. Throws ios_base::failure if an error occurs.
     */
    void db_prepare(sqlite3* & db, sqlite3_stmt* & stmt, const boost::format & sql);
 
    /** \brief Make a step for a prepared SQL statement.
     *
     * Delegates to sqlite3_step. Throws ios_base::failure if an error occurs.
     */   
    void db_step(sqlite3_stmt* & stmt, int expected_status);

    /** \brief Closes a SQLite database.
     *
     * Delegates to sqlite3_open. Throws ios_base::failure if an error occurs.
     */
    void db_close(sqlite3* & db);

}

#endif
