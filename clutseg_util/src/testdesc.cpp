/*
 * Author: Julius Adorf
 */

#include "testdesc.h"

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <boost/algorithm/string.hpp>

using namespace std;

// TODO: does not really parse a python configuration file
TestDesc loadTestDesc(const string & filename) {
    TestDesc m;
    fstream f;
    f.open(filename.c_str(), ios_base::in); 
    size_t s = 1024;
    char cline[1024];
    while (!f.eof()) {
        // getline stops on eof, no character will be
        // read twice.
        f.getline(cline, s);
        string line(cline);
        size_t offs = line.find_first_of('='); 
        if (offs != string::npos) {
            string key = line.substr(0, offs);
            string val = line.substr(offs+1, line.length() - offs);
            boost::trim(key);
            vector<string> v;
            boost::split(v, val, boost::is_any_of(" "), boost::token_compress_on);
            set<string> subjects;
            for (size_t i = 0; i < v.size(); i++) {
                boost::trim(v[i]);
                if (v[i].length() > 0) {
                    subjects.insert(v[i]);
                }
            } 
            m[key] = subjects;
        }
    }
    f.close();
    return m; 
}

