/**
 * Author: Julius Adorf
 */

#include "clutseg/runner.h"

#include "clutseg/clutseg.h"
#include "clutseg/experiment.h"
#include "clutseg/paramsel.h"
#include "clutseg/ranking.h"

#include <boost/foreach.hpp>
#include <cv.h>
#include <string>
#include <tod/detecting/Parameters.h>

using namespace cv;
using namespace std;
using namespace tod;

namespace clutseg {

    ExperimentRunner::ExperimentRunner(sqlite3* db,
                                       const TrainFeaturesCache & cache) :
                                        db_(db), cache_(cache) {}

    void ExperimentRunner::run() {
        vector<Experiment> exps;
        while (true) {
            selectExperimentsNotRun(db_, exps);
            if (exps.empty()) {
                sleep(1);
                continue;
            }
            sortExperimentsByTrainFeatures(exps);
            string current_train_set;
            string current_sha1;
            ClutSegmenter segmenter;
            BOOST_FOREACH(Experiment & exp, exps) {
                TrainFeatures tr_feat(exp.train_set, exp.paramset.train_pms_fe);
                if (exp.train_set != current_train_set || sha1(tr_feat.fe_params) != current_sha1) {
                    // check whether the train features have to be generated
                    if (!cache_.trainFeaturesExist(tr_feat)) {
                        // TODO: cache_.generateTrainFeatures(tr_feat)
                        tr_feat.generate();
                        cache_.addTrainFeatures(tr_feat);
                    }
                    segmenter = ClutSegmenter(
                        cache_.trainFeaturesDir(tr_feat).string(),
                        TODParameters(), TODParameters());
                }
                // TODO: extract method configureSegmenter
                // TODO: extract method mustReload

                // Online change configuration
                segmenter.getDetectParams().feParams = exp.paramset.recog_pms_fe;
                segmenter.getDetectParams().matcherParams = exp.paramset.detect_pms_match;
                segmenter.getDetectParams().guessParams = exp.paramset.detect_pms_guess;
                // Actually the locator does not use the feParams, this is just to
                // avoid confusion, and the lack of a distinctive null like in Java.
                segmenter.getLocateParams().feParams = exp.paramset.recog_pms_fe;
                segmenter.getLocateParams().matcherParams = exp.paramset.detect_pms_match;
                segmenter.getLocateParams().guessParams = exp.paramset.detect_pms_guess;

                // TODO: move to ranking.h / ranking.cpp
                Ptr<GuessRanking> ranking;
                if (exp.paramset.pms_clutseg.ranking == "InliersRanking") {
                    ranking = new InliersRanking();
                } else if (exp.paramset.pms_clutseg.ranking == "ProximityRanking") {
                    ranking = new ProximityRanking();
                } else {
                    cerr << "WARNING: invalid ranking: " << exp.paramset.pms_clutseg.ranking << endl;
                    continue;
                }
                segmenter.setRanking(ranking);
                segmenter.setAcceptThreshold(exp.paramset.pms_clutseg.accept_threshold);

                // for all images run segmenter.recog
                // FIXME:
                // TODO: check which statistics might be useful and cannot be generated afterwards

                // run the experiment
                // populate the response and results
                exp.serialize(db_);
            }
        }
    }

}

