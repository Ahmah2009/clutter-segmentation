/*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/map.h"

#include <tod/detecting/Loader.h>
#include <boost/foreach.hpp>
#include <limits>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace tod;

namespace clutseg {

    Clutsegmenter::Clutsegmenter() : initialized_(false) {}
        
    Clutsegmenter::Clutsegmenter(const string & baseDirectory,
                                    const string & detect_config,
                                    const string & locate_config,
                                    Ptr<GuessRanking> ranking,
                                    float accept_threshold,
                                    bool do_locate) :
                                baseDirectory_(baseDirectory),
                                ranking_(ranking),
                                accept_threshold_(accept_threshold),
                                do_locate_(do_locate),
                                initialized_(true) {
        loadParams(detect_config, detect_params_);
        loadParams(locate_config, locate_params_);
        loadBase();
    }

    Clutsegmenter::Clutsegmenter(const string & baseDirectory,
                                    const TODParameters & detect_params,
                                    const TODParameters & locate_params,
                                    Ptr<GuessRanking> ranking,
                                    float accept_threshold,
                                    bool do_locate) :
                                baseDirectory_(baseDirectory),
                                detect_params_(detect_params),
                                locate_params_(locate_params),
                                ranking_(ranking),
                                accept_threshold_(accept_threshold),
                                do_locate_(do_locate),
                                initialized_(true) {
        loadBase();
    }

    void Clutsegmenter::loadParams(const string & config, TODParameters & params) {
        FileStorage fs(config, FileStorage::READ);
        if (!fs.isOpened()) {
            throw ios_base::failure("Cannot read configuration file '" + config + "'");
        }
        params.read(fs[TODParameters::YAML_NODE_NAME]);
        fs.release();
    }

    void Clutsegmenter::loadBase() {
        Loader loader(baseDirectory_);
        loader.readTexturedObjects(objects_);
        base_ = TrainingBase(objects_);
    }

    TODParameters & Clutsegmenter::getDetectParams() {
        return detect_params_;
    }

    TODParameters & Clutsegmenter::getLocateParams() {
        return locate_params_;
    }

    int Clutsegmenter::getAcceptThreshold() const {
        return accept_threshold_;
    }

    void Clutsegmenter::setAcceptThreshold(int accept_threshold) {
        accept_threshold_ = accept_threshold;
    }

    Ptr<GuessRanking> Clutsegmenter::getRanking() const {
        return ranking_;
    }

    void Clutsegmenter::setRanking(const Ptr<GuessRanking> & ranking) {
        ranking_ = ranking;
    }

    set<string> Clutsegmenter::getTemplateNames() const {
        set<string> s;
        BOOST_FOREACH(const Ptr<TexturedObject> & t, objects_) {
            s.insert(t->name);
        }
        return s;
    }

    void Clutsegmenter::reconfigure(const Paramset & paramset) {
        detect_params_ = paramset.toDetectTodParameters();
        locate_params_ = paramset.toLocateTodParameters();
        string r = paramset.pms_clutseg.ranking;
        if (r == "InliersRanking") {
            ranking_ = new InliersRanking(); 
        } else if (r == "ProximityRanking") {
            ranking_ = new ProximityRanking();
        } else if (r == "UniformRanking") {
            ranking_ = new UniformRanking();
        } else {
            throw runtime_error("Unknown ranking: " + r);
        }
        accept_threshold_ = paramset.pms_clutseg.accept_threshold;
    }

    void Clutsegmenter::resetStats() {
        stats_ = ClutsegmenterStats();
    }

    ClutsegmenterStats Clutsegmenter::getStats() const {
        return stats_;
    }

    void Clutsegmenter::setDoLocate(bool do_locate) {
        do_locate_ = do_locate;
    }

    bool Clutsegmenter::isDoLocate() const {
        return do_locate_;
    }

    void initRecognizer(Ptr<Recognizer> & recognizer, Ptr<Matcher> & matcher, TrainingBase & base, TODParameters params, const string & baseDirectory) {
        matcher = Matcher::create(params.matcherParams);
        matcher->add(base);
        recognizer = new KinectRecognizer(&base, matcher,
                            &params.guessParams, 0 /* verbose */,
                             baseDirectory);
    }

    void Clutsegmenter::recognize(const ClutsegQuery & query, Result & result) {
        { /* begin statistics */ 
            stats_.queries++;
        } /* end statistics */

        Features2d f2d;
        f2d.image = query.img;
        
        // For statistics, we need access to the matchers at this level.
        Ptr<Matcher> detectMatcher = NULL;
        Ptr<Matcher> locateMatcher = NULL;

        // Generate a couple of guesses. Ideally, each object on the scene is
        // detected and there are no misclassifications.
        detect(f2d, result.detect_choices, detectMatcher);

        result.features = f2d;

        BOOST_FOREACH(Guess & g, result.detect_choices) {
            mapInliersToCloud(g.inlierCloud, g, query.img, query.cloud);
        }

        if (!result.detect_choices.empty()) {
            // Sort the guesses according to the ranking function.
            sort(result.detect_choices.begin(), result.detect_choices.end(), GuessComparator(ranking_));
            // Iterate over every guess, beginning with the highest ranked
            // guess. If the guess resulting of locating the object got a score
            // larger than the acceptance threshold, this is our best guess.
            for (size_t i = 0; i < result.detect_choices.size(); i++) {
                result.locate_choice = result.detect_choices[i]; 

                if (do_locate_) {
                    locate(f2d, query.cloud, result.locate_choice, locateMatcher);
                }

                cout << "[CLUTSEG] ranking: " << (*ranking_)(result.locate_choice) << endl;
                cout << "[CLUTSEG] accept_threshold: " << accept_threshold_ << endl;

                if ((*ranking_)(result.locate_choice) >= accept_threshold_) {
                    cout << "[CLUTSEG] Inliers before:  " << result.detect_choices[i].inliers.size() << ", and after: " << result.locate_choice.inliers.size() << endl;

                    { /* begin statistics */ 
                        vector<pair<int, int> > ds; 
                        detectMatcher->getLabelSizes(ds);
                        stats_.acc_detect_choice_matches += ds[result.detect_choices[i].getObject()->id].second;
                        stats_.acc_detect_choice_inliers += result.detect_choices[i].inliers.size();
                        if (do_locate_) {
                            vector<pair<int, int> > ls; 
                            locateMatcher->getLabelSizes(ls);
                            stats_.acc_locate_choice_matches += ls[result.locate_choice.getObject()->id].second;
                            stats_.acc_locate_choice_inliers += result.locate_choice.inliers.size();
                        } 
                        stats_.choices++;
                    } /* end statistics */

                    result.guess_made = true;
                    break;
                }
            }
        }
    }

    int sum_matches(Ptr<Matcher> & matcher) {
        vector<pair<int, int> > labelSizes;
        matcher->getLabelSizes(labelSizes);
        int total = 0;
        for (size_t i = 0; i < labelSizes.size(); i++) {
            total += labelSizes[i].second;
        }
        return total;
    }

    bool Clutsegmenter::detect(Features2d & queryF2d, vector<Guess> & detect_choices, Ptr<Matcher> & detectMatcher) {
        Ptr<FeatureExtractor> extractor = FeatureExtractor::create(detect_params_.feParams);
        Ptr<Recognizer> recognizer = NULL;
        initRecognizer(recognizer, detectMatcher, base_, detect_params_, baseDirectory_);

        extractor->detectAndExtract(queryF2d);
        recognizer->match(queryF2d, detect_choices);

        { /* begin statistics */
            stats_.acc_keypoints += queryF2d.keypoints.size();
            stats_.acc_detect_matches += sum_matches(detectMatcher);
            stats_.acc_detect_guesses += detect_choices.size();
            BOOST_FOREACH(const Guess & g, detect_choices) {
                stats_.acc_detect_inliers += g.inliers.size();
            }
        } /* end statistics */

        return detect_choices.empty();
    }

    bool Clutsegmenter::locate(const Features2d & queryF2d, const PointCloudT & queryCloud, Guess & locate_choice, Ptr<Matcher> & locateMatcher) {
        if (locate_params_.matcherParams.doRatioTest) {
            cerr << "[WARNING] RatioTest enabled for locating object" << endl;
        }
        vector<Ptr<TexturedObject> > so;
        BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects_) {
            if (obj->name == locate_choice.getObject()->name) {
                // Create a new textured object instance --- those thingies
                // cannot exist in multiple training bases, this breaks indices
                // that are tightly coupled between TrainingBase and
                // TexturedObject instances.
                Ptr<TexturedObject> newObj = new TexturedObject();
                newObj->id = 0;
                newObj->name = obj->name;
                newObj->directory_ = obj->directory_;
                newObj->stddev = obj->stddev;
                newObj->observations = obj->observations;
                so.push_back(newObj);
                break;
            }
        }
        TrainingBase single(so);

        Ptr<Recognizer> recognizer;
        initRecognizer(recognizer, locateMatcher, single, locate_params_, baseDirectory_);

        vector<Guess> guesses;
        recognizer->match(queryF2d, guesses); 

        cout << "[CLUTSEG] locate_matches: " << sum_matches(locateMatcher) << endl;
        stats_.acc_locate_matches += sum_matches(locateMatcher);
        stats_.acc_locate_guesses += guesses.size();
        BOOST_FOREACH(const Guess & g, guesses) {
            stats_.acc_locate_inliers += g.inliers.size();
        }

        if (guesses.empty()) {
            cerr << "[WARNING] No guess made in refinement!" << endl;
            return false;
        } else {
            BOOST_FOREACH(Guess & guess, guesses) {
                mapInliersToCloud(guess.inlierCloud, guess, queryF2d.image, queryCloud);
            }
            sort(guesses.begin(), guesses.end(), GuessComparator(ranking_));
            locate_choice = guesses[0]; 
            return true;
        }
    }

    void ClutsegmenterStats::populateResponse(Response & r) const {
        r.avg_keypoints = float(acc_keypoints) / queries;
        r.avg_detect_matches = float(acc_detect_matches) / queries;
        r.avg_detect_inliers = float(acc_detect_inliers) / acc_detect_guesses;
        r.avg_detect_choice_matches = float(acc_detect_choice_matches) / choices;
        r.avg_detect_choice_inliers = float(acc_detect_choice_inliers) / choices;
        r.avg_locate_matches = float(acc_locate_matches) / queries;
        r.avg_locate_inliers = float(acc_locate_inliers) / acc_detect_guesses;
        r.avg_locate_choice_matches = float(acc_locate_choice_matches) / choices;
        r.avg_locate_choice_inliers = float(acc_locate_choice_inliers) / choices;
    }

}

