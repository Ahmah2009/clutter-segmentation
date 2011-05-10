/*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/map.h"

#include <tod/detecting/Loader.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace tod;

namespace clutseg {
        
    ClutSegmenter::ClutSegmenter(const string & baseDirectory, const string & detect_config, const string & refine_config) {
        baseDirectory_ = baseDirectory;
        loadParams(detect_config, detect_params_);
        loadParams(refine_config, refine_params_);
        loadBase();
    }

    void ClutSegmenter::loadParams(const string & config, TODParameters & params) {
        FileStorage fs(config, FileStorage::READ);
        if (!fs.isOpened()) {
            throw ios_base::failure("Cannot read configuration file '" + config + "'");
        }
        params.read(fs[tod::TODParameters::YAML_NODE_NAME]);
        fs.release();
    }

    void ClutSegmenter::loadBase() {
        tod::Loader loader(baseDirectory_);
        loader.readTexturedObjects(objects_);
        base_ = tod::TrainingBase(objects_);
    }

    TODParameters & ClutSegmenter::getDetectParams() {
        return detect_params_;
    }

    TODParameters & ClutSegmenter::getRefineParams() {
        return refine_params_;
    }

    // helper function
    void init(Ptr<FeatureExtractor> & extractor, Ptr<Recognizer> & recognizer, TrainingBase & base, TODParameters params, const string & baseDirectory) {
        extractor = FeatureExtractor::create(params.feParams);
        Ptr<Matcher> rtMatcher = Matcher::create(params.matcherParams);
        rtMatcher->add(base);
        recognizer = new KinectRecognizer(&base, rtMatcher,
                            &params.guessParams, 0 /* verbose */,
                             baseDirectory);
    }

    bool ClutSegmenter::recognize(const Mat & queryImage, const PointCloudT & queryCloud, tod::Guess & resultingGuess, PointCloudT & inliersCloud) {
        // Initialize matcher and recognizer. This must be done prior to every
        // query,
        Ptr<FeatureExtractor> extractor;
        Ptr<Recognizer> recognizer;
        init(extractor, recognizer, base_, detect_params_, baseDirectory_);

        Features2d test;
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

            cout << "inliers before: " << resultingGuess.inliers.size() << endl;
            refine(test, queryCloud, resultingGuess, inliersCloud);
            cout << "inliers after:  " << resultingGuess.inliers.size() << endl;

            return true;
        }
    }

    bool ClutSegmenter::refine(const tod::Features2d & query, const PointCloudT & queryCloud, tod::Guess & resultingGuess, PointCloudT & inliersCloud) {
        if (refine_params_.matcherParams.doRatioTest) {
            cerr << "[WARNING] RatioTest enabled for refinement" << endl;
        }
        Ptr<tod::Matcher> rtMatcher = tod::Matcher::create(refine_params_.matcherParams);
        vector<Ptr<tod::TexturedObject> > so;
        BOOST_FOREACH(const Ptr<tod::TexturedObject> & obj, objects_) {
            if (obj->name == resultingGuess.getObject()->name) {
                // Create a new textured object instance --- those thingies
                // cannot exist in multiple training bases, this breaks indices
                // that are tightly coupled between TrainingBase and
                // TexturedObject instances.
                Ptr<tod::TexturedObject> newObj = new tod::TexturedObject();
                newObj->id = 0;
                newObj->name = obj->name;
                newObj->directory_ = obj->directory_;
                newObj->stddev = obj->stddev;
                newObj->observations = obj->observations;
                so.push_back(newObj);
                break;
            }
        }
        tod::TrainingBase single(so);
        rtMatcher->add(single);
        cv::Ptr<tod::Recognizer> recognizer = new tod::KinectRecognizer(&single, rtMatcher,
                                &refine_params_.guessParams, 0 /* verbose */, baseDirectory_);
        vector<tod::Guess> guesses;
        recognizer->match(query, guesses); 

        if (guesses.empty()) {
            // Why? 
            cerr << "[WARNING] No guess made in refinement!" << endl;
            return false;
        } else {
            int max_i = 0;
            size_t max_v = 0;
            for (size_t i = 0; i < guesses.size(); i++) {
                if (guesses[i].inliers.size() > max_v) {
                    max_v = guesses[i].inliers.size();
                    max_i = i;
                }
            }
             
            resultingGuess = guesses[max_i];
            inliersCloud = PointCloudT();      
            mapInliersToCloud(inliersCloud, resultingGuess, query.image, queryCloud);
            return true;
        }
    }

}

