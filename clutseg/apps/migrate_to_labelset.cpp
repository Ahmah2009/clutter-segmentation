/**
 * Author: Julius Adorf
 */

#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <boost/format.hpp>
    #include <boost/algorithm/string.hpp>
    #include <cstdio>
    #include <cv.h>
    #include <fstream>
    #include <iostream>
    #include <opencv_candidate/PoseRT.h>
    #include <vector>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace boost;
using namespace opencv_candidate;
namespace bfs = boost::filesystem;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        cerr << "Usage: migrate_to_labelset <file>" << endl;
        cerr << endl;
        return 1;
    }
    string f = argv[1];
    string tmpf = tmpnam(NULL);
    if (0 != std::system(str(boost::format("zcat %s > %s") % f % tmpf).c_str())) {
        return 1;
    }
    ifstream fin;
    fin.open(tmpf.c_str());
    string tmpf2 = tmpnam(NULL);
    ofstream fout;
    fout.open(tmpf2.c_str());
    vector<string> m;
    size_t s = 1024;
    char cline[1024];
    int n = 0;

    string objects[4] = { "assam_tea", "haltbare_milch", "jacobs_coffee", "icedtea" };
    while (!fin.eof()) {
        if (fin.fail()) {
            throw ios_base::failure("Cannot read line, failbit set!");                
        } 
        fin.getline(cline, s);
        string line(cline);
        for (size_t i = 0; i < 4; i++) {
            algorithm::replace_first(line, objects[i], str(boost::format("object-%d") % n++));
            m.push_back(objects[i]);
        }
        fout << line << endl;
    }

    fin.close();
    fout.close();
    convertLegacyPoseFileToDouble(tmpf2, tmpf2);
    FileStorage in(tmpf2, FileStorage::READ);
    FileStorage out(f, FileStorage::WRITE);
    out << "labels";
    out << "[";
    for (FileNodeIterator n_it = in.root().begin(); n_it != in.root().end(); n_it++) {
        Pose p;
        p.read(*n_it);
        int r = atoi((*n_it).name().substr(7).c_str());
        Label np(m[r], poseToPoseRT(p));
        np.write(out);
    }
    out << "]";
    in.release();
    out.release();
    remove(tmpf.c_str());
    remove(tmpf2.c_str());
    return 0;
}

