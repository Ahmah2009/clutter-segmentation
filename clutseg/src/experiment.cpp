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

using namespace boost;
using namespace std;
using namespace tod;

namespace clutseg {

    #ifdef TEST
        TrainFeaturesCache::TrainFeaturesCache() {}
    #endif

    TrainFeaturesCache::TrainFeaturesCache(const std::string & cache_dir) : cache_dir_(cache_dir) {}

    string TrainFeaturesCache::trainFeaturesDir(const TrainFeatures & train_features) {
        return str(format("%s/%s/%s") % cache_dir_ % train_features.train_set % sha1(train_features.fe_params));
    }

    bool TrainFeaturesCache::trainFeaturesExist(const TrainFeatures & train_features) {
        return filesystem::exists(trainFeaturesDir(train_features));
    }

    void TrainFeaturesCache::addTrainFeatures(const TrainFeatures & train_features) {
        if (trainFeaturesExist(train_features)) {
            throw runtime_error("train features already exist");
        } else {
            string tfd = trainFeaturesDir(train_features);
            filesystem::create_directories(tfd);
            string p(getenv("CLUTSEG_PATH"));
            filesystem::directory_iterator dir_it(str(format("%s/%s") % p % train_features.train_set));
            filesystem::directory_iterator dir_end;
            while (dir_it != dir_end) {
                if (filesystem::is_directory(*dir_it)) {
                    string subj = dir_it->filename();
                    filesystem::directory_iterator subj_it(*dir_it);
                    filesystem::directory_iterator subj_end;
                    filesystem::create_directory(str(format("%s/%s") % tfd % subj));
                    while (subj_it != subj_end) {
                        if (algorithm::ends_with(subj_it->filename(), ".f3d.yaml.gz")) {
                            filesystem::copy_file( *subj_it, 
                                str(format("%s/%s/%s") % tfd % subj % subj_it->filename()));
                        }
                        subj_it++;
                    }
                }
                dir_it++; 
            }
            // TODO: create or find join_path function!
            cv::FileStorage fs(str(format("%s/features.config.yaml") % tfd), cv::FileStorage::WRITE);
            fs << FeatureExtractionParams::YAML_NODE_NAME;
            train_features.fe_params.write(fs);
            fs.release();
        }
    }

    string sha1(const string & file) {
        // http://www.gnu.org/s/hello/manual/libc/Pipe-to-a-Subprocess.html
        // http://www.gnu.org/s/hello/manual/libc/Line-Input.html#Line-Input
        FILE *in;
        in = popen(str(format("sha1sum %s") % file).c_str(), "r");
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
        char * c = tmpnam(buffer);
        c = NULL;
        stringstream fn;
        fn << buffer;
        cv::FileStorage fs(fn.str(), cv::FileStorage::WRITE);
        fs << FeatureExtractionParams::YAML_NODE_NAME;
        feParams.write(fs);
        fs.release();
        string s = sha1(fn.str()); 
        filesystem::remove(fn.str());
        return s;
    }

}

