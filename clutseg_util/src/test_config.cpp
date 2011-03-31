/*
 * Author: Julius Adorf
 */

#include "test_config.h"

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <boost/algorithm/string.hpp>

using namespace std;

map<string, set<string> > loadTestConfig(const string & filename) {
    map<string, set<string> > m;
    fstream f;
    f.open(filename.c_str(), ios_base::in); 
    size_t s = 1024;
    char cline[1024];
    while (!f.eof()) {
        f.getline(cline, s);
        string line(cline);
        int offs = line.find_first_of('='); 
        string key = line.substr(0, offs);
        string val = line.substr(offs, val.length() - offs);
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
    f.close();
    return m; 
}

