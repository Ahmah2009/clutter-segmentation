/*
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"

#include "clutseg/check.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/algorithm/string.hpp>
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <cv.h>
    #include <fstream>
    #include <iostream>
    #include <set>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace opencv_candidate;
using namespace std;

namespace bfs = boost::filesystem;

namespace clutseg {

    GroundTruth  loadGroundTruth(const bfs::path & filename) {
        assert_path_exists(filename);
        GroundTruth  m = loadGroundTruthWithoutPoses(filename);
        for (GroundTruth::iterator it = m.begin(); it != m.end(); it++) {
            string img_name = it->first;   
            LabelSet g = it->second;
            string ground_name = img_name + ".ground.yaml";
            bfs::path gp = filename.parent_path() / ground_name;
            assert_path_exists(gp);
            FileStorage fs(gp.string(), FileStorage::READ);
            g.read(fs[LabelSet::YAML_NODE_NAME]);
            fs.release();
            m[img_name] = g;
        }
        return m;
    }

    GroundTruth  loadGroundTruthWithoutPoses(const bfs::path & filename) {
        assert_path_exists(filename);
        GroundTruth  m;
        ifstream f;
        f.open(filename.string().c_str()); 
        if (!f.is_open()) {
            throw ios_base::failure(
                str(boost::format(
                "Cannot open ground-truth file '%s', does it exist?") % filename));
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
                boost::trim(key);
                vector<string> v;
                boost::split(v, val, boost::is_any_of(" "), boost::token_compress_on);
                LabelSet groundTruth;
                for (size_t i = 0; i < v.size(); i++) {
                    boost::trim(v[i]);
                    if (v[i].length() > 0) {
                        Label np(v[i]);
                        groundTruth.labels.push_back(np);
                    }
                } 
                m[key] = groundTruth;
            }
        }
        f.close();
        return m; 
    }

}

