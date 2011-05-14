/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <ctype.h>
#include <cv.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <string>

using namespace boost;
using namespace std;
using namespace tod;

namespace clutseg {

    // TODO: create method that lists all training subjects

    void TrainFeatures::generate() {
        string p(getenv("CLUTSEG_PATH"));
        string train_dir = str(format("%s/%s") % p % train_set);
        filesystem::directory_iterator dir_it(train_dir);
        filesystem::directory_iterator dir_end;
        
        // As tod_training/apps/detector.cpp and
        // tod_training/apps/f3d_creator.cpp are not included in the linked
        // libraries, we cannot trigger these processes directly from C++ but
        // have to spawn a subprocess by generating a bash-script on-the-fly.
        ofstream cmd;
        cmd.open((train_dir + "/generate.bash").c_str());
        cmd << "#!/usr/bin/env bash" << endl;
        cmd << "# This file has been automatically generated. " << endl;
        cmd << "# Better do not modify it because it might be " << endl;
        cmd << "# overwritten without notification. " << endl;
        cmd << "cd $1" << endl << endl;

        while (dir_it != dir_end) {
            if (filesystem::is_directory(*dir_it)) {
                string subj = dir_it->filename();
                cmd << "echo 'extracting features for template " << subj << "'" << endl;
                cmd << "rosrun tod_training detector -d " << subj << " -j8" << endl;
                cmd << "echo '2d-3d-mapping for template " << subj << "'" << endl;
                cmd << "rosrun tod_training f3d_creator -d " << subj << " -j8" << endl;
                cmd << endl;
            }
            dir_it++;
        }

        cmd << endl;
        cmd.close();
             
        // start a subprocess
        FILE *in;
        in = popen(("bash " + train_dir + "/generate.bash " + train_dir).c_str(), "r");
        int c;
        while ((c = fgetc(in)) != EOF) {
            cout << (unsigned char) c;
        }
        fclose(in);
    }

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

    void TrainFeaturesCache::addTrainFeatures(const TrainFeatures & train_features, bool consistency_check) {
        if (trainFeaturesExist(train_features)) {
            throw runtime_error("train features already exist");
        } else {
            string p(getenv("CLUTSEG_PATH"));
            string train_dir = str(format("%s/%s") % p % train_features.train_set);

            if (consistency_check) {
                // Check whether the features.config.yaml in the training data directory matches
                // the supplied feature configuration.
                FeatureExtractionParams stored_fe_params; 
                readFeParams(str(format("%s/features.config.yaml") % train_dir), stored_fe_params);
                if (sha1(stored_fe_params) != sha1(train_features.fe_params)) {
                    throw runtime_error( str(format(
                        "Cannot add train features, feature extraction parameter mismatch detected.\n"
                        "Please make sure the features.config.yaml in the training base directory\n"
                        "matches the supplied feature configuration! This is a consistency check.\n"
                        "Checksums %s (stored) and %s (supplied)") % sha1(stored_fe_params) % sha1(train_features.fe_params)));
                }
            }

            string tfd = trainFeaturesDir(train_features);
            filesystem::create_directories(tfd);
            filesystem::directory_iterator dir_it(train_dir);
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
            
            writeFeParams(str(format("%s/features.config.yaml") % tfd), train_features.fe_params);
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
        writeFeParams(fn.str(), feParams);
        string s = sha1(fn.str()); 
        filesystem::remove(fn.str());
        return s;
    }

    void readFeParams(const string & path, FeatureExtractionParams & feParams) {
        cv::FileStorage in(path, cv::FileStorage::READ);
        feParams.read(in[FeatureExtractionParams::YAML_NODE_NAME]);
        in.release();
    }

    void writeFeParams(const std::string & path, const tod::FeatureExtractionParams & feParams) {
        cv::FileStorage out(path, cv::FileStorage::WRITE);
        out << FeatureExtractionParams::YAML_NODE_NAME;
        feParams.write(out);
        out.release();
    }

}

