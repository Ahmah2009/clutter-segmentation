/*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"
#include "clutseg/map.h"
#include <tod/detecting/Loader.h>

using namespace std;
using namespace cv;
using namespace pcl;

namespace clutseg {
        
    ClutSegmenter::ClutSegmenter(const string & baseDirectory, const string & config) {
        opts_.config = config;
        opts_.baseDirectory = baseDirectory;
        loadParams();
        loadBase();
    }

    void ClutSegmenter::loadParams() {
        FileStorage fs(opts_.config, FileStorage::READ);
        if (!fs.isOpened()) {
            throw ios_base::failure("Cannot read configuration file '" + opts_.config + "'");
        }
        opts_.params.read(fs[tod::TODParameters::YAML_NODE_NAME]);
        fs.release();
    }

    void ClutSegmenter::loadBase() {
        tod::Loader loader(opts_.baseDirectory);
        vector<Ptr<tod::TexturedObject> > objects;
        loader.readTexturedObjects(objects);
        base_ = tod::TrainingBase(objects);
    }

    bool ClutSegmenter::recognize(const Mat & queryImage, const PointCloud<PointXYZ> & queryCloud, tod::Guess & resultingGuess, PointCloud<PointXYZ> & inliersCloud) {
        // Initialize matcher and recognizer. This must be done prior to every
        // query,
        Ptr<tod::FeatureExtractor> extractor = tod::FeatureExtractor::create(opts_.params.feParams);
        Ptr<tod::Matcher> rtMatcher = tod::Matcher::create(opts_.params.matcherParams);
        rtMatcher->add(base_);
        cv::Ptr<tod::Recognizer> recognizer = new tod::KinectRecognizer(&base_, rtMatcher,
                                &opts_.params.guessParams, 0 /* verbose */, opts_.baseDirectory);

        tod::Features2d test;
        test.image = queryImage;
        vector<tod::Guess> guesses;
        extractor->detectAndExtract(test);
        recognizer->match(test, guesses);
        if (guesses.empty()) {
            return false;
        } else {
            // Select guess with most inliers. We could also use another scheme
            // to determine the best guess, for example the guess where the
            // bounding rect.  of its inliers has the longest diagonal.
            int max_i = 0;
            size_t max_v = 0;
            for (size_t i = 0; i < guesses.size(); i++) {
                if (guesses[i].inliers.size() > max_v) {
                    max_v = guesses[i].inliers.size();
                    max_i = i;
                }
            }
            resultingGuess = guesses[max_i];
          
            mapInliersToCloud(inliersCloud, resultingGuess, queryImage, queryCloud);

            // FIXME: remove outliers?

            return true;
        }
    }
}

