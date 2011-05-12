/**
 * Author: Julius Adorf
 */

#include <sqlite3.h>
#include <tod/training/feature_extraction.h>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    struct Serializable {

        virtual int serialize(sqlite3* db);

        virtual void deserialize(sqlite3* db, int id);

    };

    struct ClutsegParams {

        int accept_threshold;
        std::string ranking;

    };

    struct Paramset : public Serializable {

        tod::FeatureExtractionParams train_pms_fe;
        tod::FeatureExtractionParams recog_pms_fe;
        tod::MatcherParameters detect_pms_match;
        tod::GuessGeneratorParameters detect_pms_guess;
        tod::MatcherParameters locate_pms_match;
        tod::GuessGeneratorParameters locate_pms_guess;
        ClutsegParams pms_clutseg;

    };

    struct Response : public Serializable {

        float value;

    };

    struct Experiment : public Serializable {
       
        Paramset paramset; 
        Response response;
        std::string train_set;
        std::string test_set;
        time_t time;

    };

}

