/*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/map.h"

#include "clutseg/gcc_diagnostic_disable.h"
#include <tod/detecting/Loader.h>
#include <boost/foreach.hpp>
#include <limits>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace tod;

namespace clutseg {

    Clutsegmenter::Clutsegmenter() : initialized_(false) {}
        
    Clutsegmenter::Clutsegmenter(const string & baseDirectory,
                                    const string & detect_config,
                                    const string & refine_config,
                                    Ptr<GuessRanking> ranking,
                                    float accept_threshold,
                                    bool do_refine) :
                                baseDirectory_(baseDirectory),
                                ranking_(ranking),
                                accept_threshold_(accept_threshold),
                                do_refine_(do_refine),
                                initialized_(true) {
        loadParams(detect_config, detect_params_);
        loadParams(refine_config, refine_params_);
        loadBase();
    }

    Clutsegmenter::Clutsegmenter(const string & baseDirectory,
                                    const TODParameters & detect_params,
                                    const TODParameters & refine_params,
                                    Ptr<GuessRanking> ranking,
                                    float accept_threshold,
                                    bool do_refine) :
                                baseDirectory_(baseDirectory),
                                detect_params_(detect_params),
                                refine_params_(refine_params),
                                ranking_(ranking),
                                accept_threshold_(accept_threshold),
                                do_refine_(do_refine),
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

    TODParameters & Clutsegmenter::getRefineParams() {
        return refine_params_;
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
        refine_params_ = paramset.toLocateTodParameters();
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

    void Clutsegmenter::setDoRefine(bool do_refine) {
        do_refine_ = do_refine;
    }

    bool Clutsegmenter::isDoRefine() const {
        return do_refine_;
    }

    void Clutsegmenter::recognize(const ClutsegQuery & query, Result & result) {
        { /* begin statistics */ 
            stats_.queries++;
        } /* end statistics */

        Features2d f2d;
        f2d.image = query.img;
        
        // Generate a couple of guesses. Ideally, each object on the scene is
        // detected and there are no misclassifications.
        vector<pair<int, int> > ds;
        detect(f2d, result.detect_choices, ds);

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

                vector<pair<int, int> > ls; 
                if (do_refine_) {
                    refine(f2d, query.cloud, result.locate_choice, ls);
                }

                cout << "[CLUTSEG] ranking: " << (*ranking_)(result.locate_choice) << endl;
                cout << "[CLUTSEG] accept_threshold: " << accept_threshold_ << endl;

                if ((*ranking_)(result.locate_choice) >= accept_threshold_) {
                    cout << "[CLUTSEG] Inliers before:  " << result.detect_choices[i].inliers.size() << ", and after: " << result.locate_choice.inliers.size() << endl;

                    { /* begin statistics */ 
                        stats_.acc_detect_choice_matches += ds[result.detect_choices[i].getObject()->id].second;
                        stats_.acc_detect_choice_inliers += result.detect_choices[i].inliers.size();
                        if (do_refine_) {
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

    bool Clutsegmenter::detect(Features2d & queryF2d, vector<Guess> & detect_choices, vector<pair<int, int> > & matches) {
        Ptr<FeatureExtractor> extractor = FeatureExtractor::create(detect_params_.feParams);
        extractor->detectAndExtract(queryF2d);

        Ptr<Matcher> detectMatcher = Matcher::create(detect_params_.matcherParams);
        detectMatcher->add(base_);

        Ptr<Recognizer> recognizer = new KinectRecognizer(&base_, detectMatcher,
                            &detect_params_.guessParams, 0,
                             baseDirectory_);

        recognizer->match(queryF2d, detect_choices);
        detectMatcher->getLabelSizes(matches);

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

    bool Clutsegmenter::refine(const Features2d & queryF2d, const PointCloudT & queryCloud, Guess & refineChoice, vector<pair<int, int> > & matches) {
        if (refine_params_.matcherParams.doRatioTest) {
            cerr << "[WARNING] RatioTest enabled for locating object" << endl;
        }
        vector<Ptr<TexturedObject> > so;
        BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects_) {
            if (obj->name == refineChoice.getObject()->name) {
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

        Ptr<Matcher> refineMatcher = Matcher::create(refine_params_.matcherParams);
        refineMatcher->add(single);

        Ptr<Recognizer> recognizer = new KinectRecognizer(&single, refineMatcher,
                            &refine_params_.guessParams, 0,
                             baseDirectory_);

        vector<Guess> guesses;
        recognizer->match(queryF2d, guesses); 
        refineMatcher->getLabelSizes(matches);

        cout << "[CLUTSEG] refine_matches: " << sum_matches(refineMatcher) << endl;
        stats_.acc_locate_matches += sum_matches(refineMatcher);
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
            refineChoice = guesses[0]; 
            return true;
        }
    }

    void ClutsegmenterStats::populateResponse(Response & r) const {
        r.avg_keypoints = float(acc_keypoints) / queries;
        r.avg_detect_guesses = float(acc_detect_guesses) / queries;
        r.avg_detect_matches = float(acc_detect_matches) / queries;
        r.avg_detect_inliers = float(acc_detect_inliers) / acc_detect_guesses;
        r.avg_detect_choice_matches = float(acc_detect_choice_matches) / choices;
        r.avg_detect_choice_inliers = float(acc_detect_choice_inliers) / choices;
        r.avg_locate_guesses = float(acc_locate_guesses) / queries;
        r.avg_locate_matches = float(acc_locate_matches) / queries;
        r.avg_locate_inliers = float(acc_locate_inliers) / acc_locate_guesses;
        r.avg_locate_choice_matches = float(acc_locate_choice_matches) / choices;
        r.avg_locate_choice_inliers = float(acc_locate_choice_inliers) / choices;
    }

}

