/**
 * Author: Julius Adorf
 */

#include <boost/format.hpp>
#include <map>
#include <sqlite3.h>
#include <tod/training/feature_extraction.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Parameters.h>

namespace clutseg {


    /** A serializable object. It corresponds to a row in a relational database
     * with rowid id. If id <= 0, then the object is new and has not yet been
     * written to or read from the database. An object must manage its own
     * serialization and if it contains members that are not of type Serializable,
     * it must also manage foreign key relationships. */
    struct Serializable {

        /** Corresponds to Sqlite3 rowid. */
        int64_t id;

        Serializable() : id(-1) {}

        /** Inserts or updates corresponding rows in the database. */
        virtual void serialize(sqlite3* db);

        /** Reads member values from database. */
        virtual void deserialize(sqlite3* db);

    };

    struct ClutsegParams : public Serializable {

        float accept_threshold;
        std::string ranking;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    struct Paramset : public Serializable {

        tod::FeatureExtractionParams train_pms_fe;
        tod::FeatureExtractionParams recog_pms_fe;
        tod::MatcherParameters detect_pms_match;
        tod::GuessGeneratorParameters detect_pms_guess;
        tod::MatcherParameters locate_pms_match;
        tod::GuessGeneratorParameters locate_pms_guess;
        ClutsegParams pms_clutseg;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    struct Response : public Serializable {

        float value;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    struct Experiment : public Serializable {
       
        Paramset paramset; 
        Response response;
        std::string train_set;
        std::string test_set;
        std::string time;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    typedef std::map<std::string, std::string> MemberMap;
   
    // TODO: use template?
    void setMemberField(MemberMap & m, const std::string & field, float val);
    void setMemberField(MemberMap & m, const std::string & field, double val);
    void setMemberField(MemberMap & m, const std::string & field, int val);
    void setMemberField(MemberMap & m, const std::string & field, int64_t val);
    void setMemberField(MemberMap & m, const std::string & field, bool val);
    void setMemberField(MemberMap & m, const std::string & field, const std::string & val);

    /** No-frills, low-level insert-or-update function that generates a SQL statement. Be careful,
     * not much efforts have been spent into making it safe in any way (you can easily delete the whole
     * table by a crafted SQL-query. Also, be careful with any string data because it is not correctly
     * quoted. */
    // TODO: fix insertOrUpdate and use sqlite3_prepare_v2
    void insertOrUpdate(sqlite3* & db, const std::string & table, const MemberMap & m, int64_t & id);

}

