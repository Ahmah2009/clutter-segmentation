/*
 * Author: Julius Adorf
 */

#include "clutseg/testdesc.h"

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

namespace clutseg {

    // TODO: does not really parse a python configuration file
    TestDesc loadTestDesc(const string & filename) {
        TestDesc m;
        ifstream f;
        f.open(filename.c_str()); 
        if (!f.is_open()) {
            throw ios_base::failure(
                str(format(
                "Cannot open testdesc file '%s', does it exist?") % filename));
        }
        size_t s = 1024;
        char cline[1024];
        while (!f.eof()) {
            if (f.fail()) {
                throw ios_base::failure("Cannot read line from testdesc file, failbit set!");                
            } 
            // getline stops on eof, no character will be
            // read twice.
            f.getline(cline, s);
            string line(cline);
            size_t offs = line.find_first_of('='); 
            if (offs != string::npos) {
                string key = line.substr(0, offs);
                string val = line.substr(offs+1, line.length() - offs);
                trim(key);
                vector<string> v;
                split(v, val, is_any_of(" "), token_compress_on);
                set<string> subjects;
                for (size_t i = 0; i < v.size(); i++) {
                    trim(v[i]);
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

}

