/**
 * Author: Julius Adorf
 */

#ifndef _PARAMSEL_H_
#define _PARAMSEL_H_

#include <boost/format.hpp>
#include <map>
#include <sqlite3.h>
#include <tod/training/feature_extraction.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Parameters.h>
#include <vector>

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

        Paramset() : train_pms_fe_id(-1), recog_pms_fe_id(-1),
                    detect_pms_match_id(-1), detect_pms_guess_id(-1),
                    locate_pms_match_id(-1), locate_pms_guess_id(-1) {}
    
        tod::FeatureExtractionParams train_pms_fe;
        tod::FeatureExtractionParams recog_pms_fe;
        tod::MatcherParameters detect_pms_match;
        tod::GuessGeneratorParameters detect_pms_guess;
        tod::MatcherParameters locate_pms_match;
        tod::GuessGeneratorParameters locate_pms_guess;

        // The row identifiers are always stored explicitly in each
        // serializable structure. Since the tod::* structures are not
        // serializable, we manage ids for them externally.
        int64_t train_pms_fe_id;
        int64_t recog_pms_fe_id;
        int64_t detect_pms_match_id;
        int64_t detect_pms_guess_id;
        int64_t locate_pms_match_id;
        int64_t locate_pms_guess_id;
 
        ClutsegParams pms_clutseg;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    struct Response : public Serializable {

        Response() : value(0.0) {}

        float value;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    struct Experiment : public Serializable {
      
        Experiment() : has_run(false) {} // TODO:
 
        Paramset paramset; 
        Response response;
        std::string train_set;
        std::string test_set;
        std::string time;
        std::string vcs_commit;

        /** Specifies whether this experiment has already been carried out. In case
         * it has been carried out, and the experiment is serialized to the database
         * column response_id will be a valid reference into table response. If not
         * run yet, column response_id will be set to NULL when serializing. */
        bool has_run;

        virtual void serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db);

    };

    void deserialize_pms_fe(sqlite3* db, tod::FeatureExtractionParams & pms_fe, int64_t & id);
    void serialize_pms_fe(sqlite3* db, const tod::FeatureExtractionParams & pms_fe, int64_t & id);

    void deserialize_pms_match(sqlite3* db, tod::MatcherParameters & pms_match, int64_t & id);
    void serialize_pms_match(sqlite3* db, const tod::MatcherParameters & pms_match, int64_t & id);

    void deserialize_pms_guess(sqlite3* db, tod::GuessGeneratorParameters & pms_guess, int64_t & id);
    void serialize_pms_guess(sqlite3* db, const tod::GuessGeneratorParameters & pms_guess, int64_t & id);

    bool getVcsCommit(std::string & vcs_commit);

    void selectExperimentsNotRun(sqlite3* & db, std::vector<Experiment> & exps);

    void sortExperimentsByTrainFeatures(std::vector<Experiment> & exps);
    
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

#endif
