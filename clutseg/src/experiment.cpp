/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <ctype.h>
#include <cv.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <string>

using namespace cv;
using namespace std;
using namespace tod;

namespace clutseg {

    #ifdef TEST
        TrainCache::TrainCache() {}
    #endif

    TrainCache::TrainCache(const std::string & cache_dir) : cache_dir_(cache_dir) {}

    string TrainCache::trainFeaturesDir(const string & train_set, const FeatureExtractionParams & feParams) {
        return str(boost::format("%s/%s/%s") % cache_dir_ % train_set % sha1(feParams));
    }

    bool TrainCache::trainFeaturesExist(const string & train_set, const FeatureExtractionParams & feParams) {
        return boost::filesystem::exists(trainFeaturesDir(train_set, feParams));
    }

    void TrainCache::addTrainFeatures(const string & train_set, const FeatureExtractionParams & feParams) {
        if (trainFeaturesExist(train_set, feParams)) {
            throw runtime_error("train features already exist");
        } else {
            boost::filesystem::create_directories(trainFeaturesDir(train_set, feParams));
            string p(getenv("CLUTSEG_PATH"));
            string train_dir = str(boost::format("%s/%s") % p % train_set);
            boost::filesystem::directory_iterator dir_it(train_dir);
            boost::filesystem::directory_iterator dir_end;
            while (dir_it != dir_end) {
                if (boost::filesystem::is_directory(*dir_it)) {
                    cout << *dir_it << endl;
                    boost::filesystem::directory_iterator subj_it(*dir_it);
                    boost::filesystem::directory_iterator subj_end;
                    boost::filesystem::create_directory(str(
                        boost::format("%s/%s") % trainFeaturesDir(train_set, feParams) % dir_it->filename()));
                    while (subj_it != subj_end) {
                        if (boost::algorithm::ends_with(subj_it->filename(), ".f3d.yaml.gz")) {
                            cout << "    " << *subj_it << endl;
                            boost::filesystem::copy_file(
                                *subj_it,
                                str(boost::format("%s/%s/%s") % trainFeaturesDir(train_set, feParams) % dir_it->filename() % subj_it->filename()));
                        }
                        subj_it++;
                    }
                }
                dir_it++; 
            }
        }
    }

    string sha1(const string & file) {
        // http://www.gnu.org/s/hello/manual/libc/Pipe-to-a-Subprocess.html
        // http://www.gnu.org/s/hello/manual/libc/Line-Input.html#Line-Input
        FILE *in;
        in = popen(str(boost::format("sha1sum %s") % file).c_str(), "r");
        char *line = NULL;
        size_t n = 0;
        ssize_t len = getline(&line, &n, in);
        pclose(in);
        stringstream s;
        for (ssize_t i = 0; i < len && !isspace(line[i]); i++) {
            s << line[i];
        }
        return s.str();
    }

    string sha1(const FeatureExtractionParams & feParams) {
        char buffer[L_tmpnam];
        // TODO: get rid of annoying warning
        // annoying, code has been taken from C++ Reference
        // and it produces a warning ...
        tmpnam(buffer);
        stringstream fn;
        fn << buffer;
        FileStorage fs(fn.str(), FileStorage::WRITE);
        fs << FeatureExtractionParams::YAML_NODE_NAME;
        feParams.write(fs);
        fs.release();
        string s = sha1(fn.str()); 
        boost::filesystem::remove(fn.str());
        return s;
    }

}

