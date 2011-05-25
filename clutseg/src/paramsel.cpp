/**
 * Author: Julius Adorf
 */

#include "clutseg/paramsel.h"

#include "clutseg/experiment.h"
#include "clutseg/db.h"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <ctime>
#include <iostream>
#include <map>

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
            "select detector_type, extractor_type, descriptor_type, "
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
        setMemberField(m, "locate_sipc", locate_sipc.final_score);
        setMemberField(m, "locate_sipc_rscore", locate_sipc.rscore);
        setMemberField(m, "locate_sipc_tscore", locate_sipc.tscore);
        setMemberField(m, "locate_sipc_cscore", locate_sipc.cscore);
        setMemberField(m, "locate_sipc_max_rscore", locate_sipc.max_rscore);
        setMemberField(m, "locate_sipc_max_tscore", locate_sipc.max_tscore);
        setMemberField(m, "locate_sipc_max_cscore", locate_sipc.max_cscore);
        setMemberField(m, "avg_angle_err", avg_angle_err);
        setMemberField(m, "avg_succ_angle_err", avg_succ_angle_err);
        setMemberField(m, "avg_trans_err", avg_trans_err);
        setMemberField(m, "avg_succ_trans_err", avg_succ_trans_err);
        setMemberField(m, "avg_angle_sq_err", avg_angle_sq_err);
        setMemberField(m, "avg_succ_angle_sq_err", avg_succ_angle_sq_err);
        setMemberField(m, "avg_trans_sq_err", avg_trans_sq_err);
        setMemberField(m, "avg_succ_trans_sq_err", avg_succ_trans_sq_err);
        setMemberField(m, "succ_rate", succ_rate);
        setMemberField(m, "mislabel_rate", mislabel_rate);
        setMemberField(m, "none_rate", none_rate);
        setMemberField(m, "avg_keypoints", avg_keypoints);
        setMemberField(m, "avg_detect_matches", avg_detect_matches);
        setMemberField(m, "avg_detect_inliers", avg_detect_inliers);
        setMemberField(m, "avg_detect_choice_matches", avg_detect_choice_matches);
        setMemberField(m, "avg_detect_choice_inliers", avg_detect_choice_inliers);
        setMemberField(m, "detect_tp", detect_tp);
        setMemberField(m, "detect_fp", detect_fp);
        setMemberField(m, "detect_fn", detect_fn);
        setMemberField(m, "detect_tn", detect_tn);
        setMemberField(m, "avg_locate_matches", avg_locate_matches);
        setMemberField(m, "avg_locate_inliers", avg_locate_inliers);
        setMemberField(m, "avg_locate_choice_matches", avg_locate_choice_matches);
        setMemberField(m, "avg_locate_choice_inliers", avg_locate_choice_inliers);
        insertOrUpdate(db, "response", m, id);
    }

    void Response::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select "
            "value, " // 0
            "locate_sipc, " // 1
            "locate_sipc_rscore, " // 2
            "locate_sipc_tscore, " // 3
            "locate_sipc_cscore ," // 4 
            "locate_sipc_max_rscore, " // 5 
            "locate_sipc_max_tscore, " // 6 
            "locate_sipc_max_cscore, " // 7 
            "avg_angle_err, " // 8 
            "avg_succ_angle_err, " // 9
            "avg_trans_err, " // 10
            "avg_succ_trans_err, " // 11
            "avg_angle_sq_err, " // 12
            "avg_succ_angle_sq_err, " // 13
            "avg_trans_sq_err, " // 14
            "avg_succ_trans_sq_err, " // 15
            "succ_rate, " // 16
            "mislabel_rate, " // 17
            "none_rate, " // 18
            "avg_keypoints, " // 19
            "avg_detect_matches, " // 20
            "avg_detect_inliers, " // 21
            "avg_detect_choice_matches, " // 22
            "avg_detect_choice_inliers, " // 23
            "detect_tp, " // 24
            "detect_fp, " // 25
            "detect_fn, " // 26
            "detect_tn, " // 27
            "avg_locate_matches, " // 28
            "avg_locate_inliers, " // 29
            "avg_locate_choice_matches, " // 30
            "avg_locate_choice_inliers " // 31
            "from response where id=%d;") % id);
        db_step(read, SQLITE_ROW);
        value = sqlite3_column_double(read, 0);
        locate_sipc.final_score = sqlite3_column_double(read, 1);
        locate_sipc.rscore = sqlite3_column_double(read, 2);
        locate_sipc.tscore = sqlite3_column_double(read, 3);
        locate_sipc.cscore = sqlite3_column_double(read, 4);
        locate_sipc.max_rscore = sqlite3_column_int(read, 5);
        locate_sipc.max_tscore = sqlite3_column_int(read, 6);
        locate_sipc.max_cscore = sqlite3_column_int(read, 7);
        avg_angle_err = sqlite3_column_double(read, 8);
        avg_succ_angle_err = sqlite3_column_double(read, 9);
        avg_trans_err = sqlite3_column_double(read, 10);
        avg_succ_trans_err = sqlite3_column_double(read, 11);
        avg_angle_sq_err = sqlite3_column_double(read, 12);
        avg_succ_angle_sq_err = sqlite3_column_double(read, 13);
        avg_trans_sq_err = sqlite3_column_double(read, 14);
        avg_succ_trans_sq_err = sqlite3_column_double(read, 15);
        succ_rate = sqlite3_column_double(read, 16);
        mislabel_rate = sqlite3_column_double(read, 17);
        none_rate = sqlite3_column_double(read, 18);
        avg_keypoints = sqlite3_column_double(read, 19);
        avg_detect_matches = sqlite3_column_double(read, 20);
        avg_detect_inliers = sqlite3_column_double(read, 21);
        avg_detect_choice_matches = sqlite3_column_double(read, 22);
        avg_detect_choice_inliers = sqlite3_column_double(read, 23);
        detect_tp = sqlite3_column_int(read, 24);
        detect_fp = sqlite3_column_int(read, 25);
        detect_fn = sqlite3_column_int(read, 26);
        detect_tn = sqlite3_column_int(read, 27);
        avg_locate_matches = sqlite3_column_double(read, 28);
        avg_locate_inliers = sqlite3_column_double(read, 29);
        avg_locate_choice_matches = sqlite3_column_double(read, 30);
        avg_locate_choice_inliers = sqlite3_column_double(read, 31);
        sqlite3_finalize(read);
    }

    void Experiment::serialize(sqlite3* db) {
        if (has_run) {
            response.serialize(db);
        }
        paramset.serialize(db);
        MemberMap m;
        setMemberField(m, "paramset_id", paramset.id);
        if (has_run) {
            setMemberField(m, "response_id", response.id);
        }
        setMemberField(m, "train_set", train_set);
        setMemberField(m, "test_set", test_set);
        if (time != "") {
            setMemberField(m, "time", time);
        }
        if (vcs_commit != "") {
            setMemberField(m, "vcs_commit", vcs_commit);
        }
        setMemberField(m, "skip", skip);
        insertOrUpdate(db, "experiment", m, id);
    }
    
    void Experiment::deserialize(sqlite3* db) {
        sqlite3_stmt *read;
        db_prepare(db, read, boost::format("select "
            "paramset_id, " // 0
            "response_id, " // 1
            "train_set, " // 2
            "test_set, " // 3
            "time, " // 4
            "vcs_commit, " // 5
            "skip " // 6
            "from experiment where id=%d;") % id);
        db_step(read, SQLITE_ROW);
       
        has_run = (sqlite3_column_type(read, 1) != SQLITE_NULL);
        paramset.id = sqlite3_column_int64(read, 0);
        if (has_run) {
            response.id = sqlite3_column_int64(read, 1);
        } else {
            response.id = -1;
        }
        train_set = string((const char*) sqlite3_column_text(read, 2));
        test_set = string((const char*) sqlite3_column_text(read, 3));
        if (has_run) {
            time = string((const char*) sqlite3_column_text(read, 4));
            vcs_commit = string((const char*) sqlite3_column_text(read, 5));
        }
        skip = sqlite3_column_int(read, 6) != 0;
        sqlite3_finalize(read);
        paramset.deserialize(db);
        if (has_run) {
            response.deserialize(db);
        }
    }

    void Experiment::record_time() {
        time_t tt = std::time(NULL);
        tm *t = localtime(&tt);
        char ts[128];
        strftime(ts, 128, "%Y-%m-%d %H:%M:%S", t);
        time = string(ts);
    }

    void Experiment::record_commit() {
        FILE *in;
        in = popen(str(boost::format("git log -1 --format=oneline %s/clutter-segmentation") % getenv("CLUTSEG_PATH")).c_str(), "r");
        char *line = NULL;
        size_t n = 0;
        ssize_t len = getline(&line, &n, in);
        stringstream s;
        for (ssize_t i = 0; i < len; i++) {
            s << line[i];
        }
        pclose(in);
        size_t offs = s.str().find(" ");
        if (s.str() != "" && offs != string::npos && offs == 40) {
            vcs_commit = s.str().substr(0, offs);
        } else {
            vcs_commit = "";
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
        sqlite3_stmt *select;
        string sql = "select id from experiment where response_id is null;";
        db_prepare(db, select, sql);
        // TODO: clear interface for logging sql statements, is there any hook in sqlite3?
        cout << "[SQL] " << sql << endl;
        vector<int> ids;
        while (sqlite3_step(select) == SQLITE_ROW) {
            ids.push_back(sqlite3_column_int(select, 0));
        }
        exps.resize(ids.size());
        for (size_t i = 0; i < ids.size(); i++) {
            exps[i].id = ids[i];
            exps[i].deserialize(db);
        } 
        sqlite3_finalize(select);
    }

    struct ExperimentTrainFeaturesComparator {
        bool operator()(const Experiment & a, const Experiment & b) {
            return (a.train_set == b.train_set) ?
                (sha1(a.paramset.train_pms_fe) < sha1(b.paramset.train_pms_fe)) :
                (a.train_set < b.train_set);
        }
    };

    void sortExperimentsByTrainFeatures(std::vector<Experiment> & exps) {
        sort(exps.begin(), exps.end(), ExperimentTrainFeaturesComparator());
    }

}

