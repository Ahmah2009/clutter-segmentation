/**
 * Author: Julius Adorf
 */

#include <sqlite3.h>
#include <tod/training/feature_extraction.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Parameters.h>

namespace clutseg {

    struct Serializable {

        virtual int64_t serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db, int64_t id);

    };

    struct ClutsegParams {

        float accept_threshold;
        std::string ranking;

        virtual int64_t serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db, int64_t id);

    };

    struct Paramset : public Serializable {

        tod::FeatureExtractionParams train_pms_fe;
        tod::FeatureExtractionParams recog_pms_fe;
        tod::MatcherParameters detect_pms_match;
        tod::GuessGeneratorParameters detect_pms_guess;
        tod::MatcherParameters locate_pms_match;
        tod::GuessGeneratorParameters locate_pms_guess;
        ClutsegParams pms_clutseg;

        virtual int64_t serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db, int64_t id);

    };

    struct Response : public Serializable {

        float value;

        virtual int64_t serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db, int64_t id);

    };

    struct Experiment : public Serializable {
       
        Paramset paramset; 
        Response response;
        std::string train_set;
        std::string test_set;
        time_t time;

        virtual int64_t serialize(sqlite3* db);
        virtual void deserialize(sqlite3* db, int64_t id);

    };

}

