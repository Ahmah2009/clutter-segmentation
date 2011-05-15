/**
 * Author: Julius Adorf
 */

#include "clutseg/paramsel.h"
#include "clutseg/db.h"

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <map>
#include <utility>

using namespace std;
using namespace tod;

namespace clutseg {

    void Serializable::serialize(sqlite3* db) {
        throw ios_base::failure("Cannot serialize, not implemented.");
    }

    void Serializable::deserialize(sqlite3* db) {
        throw ios_base::failure("Cannot deserialize, not implemented.");
    }

    void ClutsegParams::serialize(sqlite3* db) {
        MemberMap m;
        setMemberField(m, "accept_threshold", accept_threshold);
        setMemberField(m, "ranking", ranking);
        insertOrUpdate(db, "pms_clutseg", m, id);
    }

    void ClutsegParams::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select accept_threshold, ranking from pms_clutseg where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        accept_threshold = sqlite3_column_double(read, 0);
        ranking = string((const char*) sqlite3_column_text(read, 1));
        sqlite3_finalize(read);
    }

    void deserialize_pms_fe(sqlite3* db, FeatureExtractionParams & pms_fe, int64_t & id) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format(
            "select detector_type, extractor_type, descriptor_type "
            "threshold, min_features, max_features, octaves "
            "from pms_fe where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        pms_fe.detector_type = string((const char*) sqlite3_column_text(read, 0));
        pms_fe.extractor_type = string((const char*) sqlite3_column_text(read, 1));
        pms_fe.descriptor_type = string((const char*) sqlite3_column_text(read, 2));
        pms_fe.detector_params["threshold"] = sqlite3_column_double(read, 3);
        pms_fe.detector_params["min_features"] = sqlite3_column_int(read, 4);
        pms_fe.detector_params["max_features"] = sqlite3_column_int(read, 5);
        pms_fe.extractor_params["octaves"] = sqlite3_column_int(read, 6);
        sqlite3_finalize(read);
    }

    void serialize_pms_fe(sqlite3* db, const FeatureExtractionParams & pms_fe, int64_t & id) {
        MemberMap m;
        setMemberField(m, "detector_type", pms_fe.detector_type);
        setMemberField(m, "extractor_type", pms_fe.extractor_type);
        setMemberField(m, "descriptor_type", pms_fe.descriptor_type);
        setMemberField(m, "threshold", (double) pms_fe.detector_params.find("threshold")->second);
        setMemberField(m, "min_features", (int) pms_fe.detector_params.find("min_features")->second);
        setMemberField(m, "max_features", (int) pms_fe.detector_params.find("max_features")->second);
        setMemberField(m, "octaves", (int) pms_fe.extractor_params.find("octaves")->second);
        insertOrUpdate(db, "pms_fe", m, id);
    }

    void deserialize_pms_match(sqlite3* db, MatcherParameters & pms_match, int64_t & id) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format(
            "select matcher_type, knn, do_ratio_test, ratio_threshold "
            "from pms_match where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        pms_match.type = string((const char*) sqlite3_column_text(read, 0));
        pms_match.knn = sqlite3_column_int(read, 1);
        pms_match.doRatioTest = sqlite3_column_int(read, 2);
        pms_match.ratioThreshold = sqlite3_column_double(read, 3);
        sqlite3_finalize(read);
    }

    void serialize_pms_match(sqlite3* db, const MatcherParameters & pms_match, int64_t & id) {
        MemberMap m;
        setMemberField(m, "matcher_type", pms_match.type);
        setMemberField(m, "knn", pms_match.knn);
        setMemberField(m, "do_ratio_test", pms_match.doRatioTest);
        setMemberField(m, "ratio_threshold", pms_match.ratioThreshold);
        insertOrUpdate(db, "pms_match", m, id);
    }

    void deserialize_pms_guess(sqlite3* db, GuessGeneratorParameters & pms_guess, int64_t & id) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format(
            "select ransac_iterations_count, min_inliers_count, max_projection_error "
            "from pms_guess where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        pms_guess.ransacIterationsCount = sqlite3_column_int(read, 0);
        pms_guess.minInliersCount = sqlite3_column_int(read, 1);
        pms_guess.maxProjectionError = sqlite3_column_double(read, 2);
        sqlite3_finalize(read);
    }

    void serialize_pms_guess(sqlite3* db, const GuessGeneratorParameters & pms_guess, int64_t & id) {
        MemberMap m;
        setMemberField(m, "ransac_iterations_count", pms_guess.ransacIterationsCount);
        setMemberField(m, "min_inliers_count", pms_guess.minInliersCount);
        setMemberField(m, "max_projection_error", pms_guess.maxProjectionError);
        insertOrUpdate(db, "pms_guess", m, id);
    }

    void Paramset::serialize(sqlite3* db) {
        serialize_pms_fe(db, train_pms_fe, train_pms_fe_id);
        serialize_pms_fe(db, recog_pms_fe, recog_pms_fe_id);
        serialize_pms_match(db, detect_pms_match, detect_pms_match_id);
        serialize_pms_guess(db, detect_pms_guess, detect_pms_guess_id);
        serialize_pms_match(db, locate_pms_match, locate_pms_match_id);
        serialize_pms_guess(db, locate_pms_guess, locate_pms_guess_id);
        pms_clutseg.serialize(db);

        MemberMap m;
        setMemberField(m, "train_pms_fe_id", train_pms_fe_id);
        setMemberField(m, "recog_pms_fe_id", recog_pms_fe_id);
        setMemberField(m, "detect_pms_match_id", detect_pms_match_id);
        setMemberField(m, "detect_pms_guess_id", detect_pms_guess_id);
        setMemberField(m, "locate_pms_match_id", locate_pms_match_id);
        setMemberField(m, "locate_pms_guess_id", locate_pms_guess_id);
        setMemberField(m, "recog_pms_clutseg_id", pms_clutseg.id);
        insertOrUpdate(db, "paramset", m, id);
    }

    void Paramset::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format(
            "select train_pms_fe_id, recog_pms_fe_id,  "
            "detect_pms_match_id, detect_pms_guess_id, "
            "locate_pms_match_id, locate_pms_guess_id, "
            "recog_pms_clutseg_id from paramset where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        train_pms_fe_id = sqlite3_column_int64(read, 0);
        recog_pms_fe_id = sqlite3_column_int64(read, 1);
        detect_pms_match_id = sqlite3_column_int64(read, 2);
        detect_pms_guess_id = sqlite3_column_int64(read, 3);
        locate_pms_match_id = sqlite3_column_int64(read, 4);
        locate_pms_guess_id = sqlite3_column_int64(read, 5);
        pms_clutseg.id = sqlite3_column_int64(read, 6);
        sqlite3_finalize(read);
        
        deserialize_pms_fe(db, train_pms_fe, train_pms_fe_id);
        deserialize_pms_fe(db, recog_pms_fe, recog_pms_fe_id);
        deserialize_pms_match(db, detect_pms_match, detect_pms_match_id);
        deserialize_pms_guess(db, detect_pms_guess, detect_pms_guess_id);
        deserialize_pms_match(db, locate_pms_match, locate_pms_match_id);
        deserialize_pms_guess(db, locate_pms_guess, locate_pms_guess_id);
        pms_clutseg.deserialize(db);
    }

    void Response::serialize(sqlite3* db) {
        MemberMap m;
        setMemberField(m, "value", value);
        insertOrUpdate(db, "response", m, id);
    }

    void Response::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select value from response where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        value = sqlite3_column_double(read, 0);
        sqlite3_finalize(read);
    }

    void Experiment::serialize(sqlite3* db) {
        if (run) {
            response.serialize(db);
        }
        paramset.serialize(db);
        MemberMap m;
        setMemberField(m, "paramset_id", paramset.id);
        if (run) {
            setMemberField(m, "response_id", response.id);
        }
        setMemberField(m, "train_set", train_set);
        setMemberField(m, "test_set", test_set);
        setMemberField(m, "time", time);
        insertOrUpdate(db, "experiment", m, id);
    }
    
    void Experiment::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select paramset_id, response_id, train_set, test_set, time from experiment where id=%d;") % id);
        db_step(read, SQLITE_ROW);
       
        run = (sqlite3_column_type(read, 1) != SQLITE_NULL);
        paramset.id = sqlite3_column_int64(read, 0);
        if (run) {
            response.id = sqlite3_column_int64(read, 1);
        } else {
            response.id = -1;
        }
        train_set = string((const char*) sqlite3_column_text(read, 2));
        test_set = string((const char*) sqlite3_column_text(read, 3));
        time = string((const char*) sqlite3_column_text(read, 4));
        sqlite3_finalize(read);
        paramset.deserialize(db);
        if (run) {
            response.deserialize(db);
        }
    }

    void setMemberField(MemberMap & m, const std::string & field, float val) {
        m[field] = str(boost::format("%f") % val);
    }
    void setMemberField(MemberMap & m, const std::string & field, double val) {
        m[field] = str(boost::format("%f") % val);
    }

    void setMemberField(MemberMap & m, const std::string & field, int val) {
        m[field] = str(boost::format("%d") % val);
    }

    void setMemberField(MemberMap & m, const std::string & field, int64_t val) {
        m[field] = str(boost::format("%ld") % val);
    }

    void setMemberField(MemberMap & m, const std::string & field, bool val) {
        m[field] = str(boost::format("%s") % val);
    }

    void setMemberField(MemberMap & m, const std::string & field, const std::string & val) {
        m[field] = val;
    }

    void insertOrUpdate(sqlite3*  & db, const std::string & table, const MemberMap & m, int64_t & id) {
        if (id > 0) {
            stringstream upd;
            
            MemberMap::const_iterator it = m.begin();
            MemberMap::const_iterator end = m.end();

            upd << "update " << table << " set ";
            for (size_t i = 0; it != end; i++) {
                upd << it->first;
                upd << "=";
                upd << "'" << it->second << "'";
                if (i < m.size() - 1) {
                    upd << ", ";
                }
                it++;
            }
            upd << " where id=" << id << ";";

            db_exec(db, upd.str());
        } else {
            stringstream ins;
            
            MemberMap::const_iterator it = m.begin();
            MemberMap::const_iterator end = m.end();

            ins << "insert into " << table << " (";
            for (size_t i = 0; it != end; i++) {
                ins << it->first;
                if (i < m.size() - 1) {
                    ins << ", ";
                }
                it++;
            }
            ins << ") values (";
 
            it = m.begin();
            for (size_t i = 0; it != end; i++) {
                ins << "'" << it->second << "'";
                if (i < m.size() - 1) {
                    ins << ", ";
                }
                it++;
            }
            ins << ");";

            db_exec(db, ins.str());
            id = sqlite3_last_insert_rowid(db);
        }
    }

   void selectExperimentsNotRun(sqlite3* & db, vector<Experiment> & exps) {

    }
}

