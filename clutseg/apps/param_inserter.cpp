/**
 * Author: Julius Adorf
 */

#include "clutseg/db.h"
#include "clutseg/experiment.h"
#include "clutseg/paramsel.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/format.hpp>
    #include <iostream>
    #include <sqlite3.h>
    #include <sstream>
    #include <string>
    #include <tod/detecting/Parameters.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

string readline() {
    char linebuf[1042];
    cin.getline(linebuf, 1024);
    return string(linebuf);
}

string readvalue(const string & key, const string & defaultval) {
    cout << key << " [" << defaultval << "] > ";
    string v = readline();
    if (v == "") {
        return defaultval;
    } else {
        return v;
    }
}

struct insert_values {
    insert_values() : accept_threshold(0.0) {}
    string train_set;
    string test_set; 
    bfs::path train_pms_fe_file; 
    bfs::path detect_pms_file; 
    bfs::path locate_pms_file; 
    string ranking; 
    float accept_threshold;
};

void cmd_help() {
    cout << "Valid commands: " << endl
         << "    insert     add a new experiment. " << endl
         << "    help       show this help message. " << endl;
}

void cmd_insert(sqlite3* & db) {
    insert_values input;
    insert_values defaults;

    // TODO: read defaults from somewhere
    defaults.train_set = "ias_kinect_train_v2";
    defaults.test_set = "ias_kinect_test_train_20";
    defaults.train_pms_fe_file = bfs::path("train.features.config.yaml");
    defaults.detect_pms_file = bfs::path("detect.config.yaml");
    defaults.locate_pms_file = bfs::path("locate.config.yaml");
    defaults.ranking = "InliersRanking";
    defaults.accept_threshold = 5;
    
    input.train_set = readvalue("train_set", defaults.train_set);
    input.test_set = readvalue("test_set", defaults.test_set);
    input.train_pms_fe_file = bfs::path(readvalue("train_pms_fe_file", defaults.train_pms_fe_file.string()));
    input.detect_pms_file = bfs::path(readvalue("detect_pms_file", defaults.detect_pms_file.string()));
    input.locate_pms_file = bfs::path(readvalue("locate_pms_file", defaults.locate_pms_file.string()));
    input.ranking = readvalue("ranking", defaults.ranking);
    input.accept_threshold = atof(readvalue("accept_threshold", str(boost::format("%d") % defaults.accept_threshold)).c_str());

    Experiment exp;
    exp.train_set = input.train_set;
    exp.test_set = input.test_set;
    readFeParams(input.train_pms_fe_file, exp.paramset.train_pms_fe);
    TODParameters detect_pms;
    readTodParams(input.detect_pms_file, detect_pms);
    TODParameters locate_pms;
    readTodParams(input.locate_pms_file, locate_pms);
    exp.paramset.recog_pms_fe = detect_pms.feParams;
    exp.paramset.detect_pms_match = detect_pms.matcherParams;
    exp.paramset.detect_pms_guess = detect_pms.guessParams;
    exp.paramset.locate_pms_match = locate_pms.matcherParams;
    exp.paramset.locate_pms_guess = locate_pms.guessParams;
    exp.paramset.pms_clutseg.ranking = input.ranking;
    exp.paramset.pms_clutseg.accept_threshold = input.accept_threshold;

    exp.serialize(db);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Usage: param_insert <database>" << endl;
        return -1;
    }

    sqlite3* db;
    db_open(db, argv[1]);

    while (!cin.eof()) {
        cout << "> ";
        string line = readline();
        if (line == "help") {
            cmd_help();
        } else if (line == "insert") {
            cmd_insert(db);
        } else  {
            cerr << "Unknown command. Type 'help' for a list of valid commands." << endl;
        }
    }
}

