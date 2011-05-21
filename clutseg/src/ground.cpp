/*
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cv.h>
#include <fstream>
#include <iostream>
#include <set>

using namespace cv;
using namespace opencv_candidate;
using namespace std;

namespace bfs = boost::filesystem;

namespace clutseg {

    bool GroundTruth::onScene(const string & name) const {
        // slow 
        BOOST_FOREACH(const LabeledPose & np, labels) {
            if (np.name == name) {
                return true;
            }
        }
        return false;
    }

    int GroundTruth::distinctLabelCount() const {
        set<string> d;
        BOOST_FOREACH(const LabeledPose & np, labels) {
            d.insert(np.name);
        }
        return d.size();
    }

    vector<PoseRT> GroundTruth::posesOf(const string & subject) const {
        vector<PoseRT> ps;
        BOOST_FOREACH(const LabeledPose & np, labels) {
            if (np.name == subject) {
                ps.push_back(np.pose);
            }
        }
        return ps;
    }

    void GroundTruth::read(const bfs::path & filename) {
        cout << "[GROUND] Reading in " << filename << endl;
        labels.clear();
        FileStorage fs = FileStorage(filename.string(), FileStorage::READ);
        // iterate over objects
        for (FileNodeIterator n_it = fs.root().begin(); n_it != fs.root().end(); n_it++) {
            LabeledPose np((*n_it).name());
            np.pose.read(*n_it);
            np.pose.estimated = true;
            labels.push_back(np);  
            cout << "[GROUND] Read " << np.name << endl;
        }
    }

    void GroundTruth::write(const bfs::path & filename) {
        FileStorage fs(filename.string(), FileStorage::WRITE);
        BOOST_FOREACH(const LabeledPose & np, labels) {
            fs << np.name;
            np.pose.write(fs);
        }
        fs.release();
    } 

    SetGroundTruth loadSetGroundTruth(const bfs::path & filename) {
        SetGroundTruth m = loadSetGroundTruthWithoutPoses(filename);
        for (SetGroundTruth::iterator it = m.begin(); it != m.end(); it++) {
            string img_name = it->first;   
            GroundTruth g = it->second;
            string ground_name = img_name + ".ground.yaml";
            g.read(filename.parent_path() / ground_name);
        }
        return m;
    }

    // TODO: does not really parse a python configuration file
    SetGroundTruth loadSetGroundTruthWithoutPoses(const bfs::path & filename) {
        SetGroundTruth m;
        ifstream f;
        f.open(filename.string().c_str()); 
        if (!f.is_open()) {
            throw ios_base::failure(
                str(boost::format(
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
                boost::trim(key);
                vector<string> v;
                boost::split(v, val, boost::is_any_of(" "), boost::token_compress_on);
                GroundTruth groundTruth;
                for (size_t i = 0; i < v.size(); i++) {
                    boost::trim(v[i]);
                    if (v[i].length() > 0) {
                        LabeledPose np(v[i]);
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

