/**
 * Author: Julius Adorf
 */

#include "clutseg/runner.h"

#include "clutseg/clutseg.h"
#include "clutseg/experiment.h"
#include "clutseg/paramsel.h"
#include "clutseg/ranking.h"
#include "clutseg/response.h"
#include "clutseg/ground.h"

#include <boost/foreach.hpp>
#include <cv.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <tod/detecting/Parameters.h>

using namespace cv;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

namespace clutseg {

    ExperimentRunner::ExperimentRunner() {}

    ExperimentRunner::ExperimentRunner(sqlite3* db,
                                       const TrainFeaturesCache & cache) :
                                        db_(db), cache_(cache) {}

    bfs::path cloudPath(const bfs::path & img_path) {
        string fn = img_path.filename();
        size_t offs = fn.rfind("_");
        if (offs != string::npos) {
            size_t offs2 = fn.rfind(".");
            if (offs2 != string::npos) {
                string midfix = fn.substr(offs+1, offs2 - offs - 1);
                return img_path.parent_path() / ("cloud_" + midfix + ".pcd");
            }
        }
        return img_path.parent_path() / (img_path.filename() + ".cloud.pcd");
    }

    void ExperimentRunner::runExperiment(ClutSegmenter & segmenter, Experiment & exp) {
        // TODO: check which statistics might be useful and cannot be generated afterwards
        bfs::path p = getenv("CLUTSEG_PATH");
        bfs::path test_dir = p / exp.test_set;
        TestSetGroundTruth testdesc = loadTestSetGroundTruth(test_dir / "testdesc.txt");
        TestSetResult result;
        // Loop over all images in the test set
        for (TestSetGroundTruth::iterator it = testdesc.begin(); it != testdesc.end(); it++) {
            string img_name = it->first;
            cout << img_name << endl;
            bfs::path img_path = test_dir / img_name;
            Mat queryImage = imread(img_path.string(), 0);
            if (queryImage.empty()) {
                throw runtime_error(str(boost::format(
                    "Cannot read image '%s' for experiment with id=%d. Please check\n"
                    "whether image file exists. Full path is '%s'."
                ) % img_name % exp.id % img_path));
            }
            PointCloudT queryCloud;
            bfs::path cloud_path = cloudPath(img_path);
            if (bfs::exists(cloud_path)) {
                pcl::io::loadPCDFile(cloud_path.string(), queryCloud);
            }
            Guess guess;
            PointCloudT inliersCloud;
            bool pos = segmenter.recognize(queryImage, queryCloud, guess, inliersCloud);
            cout << "[RUN] Recognized " << (pos ? guess.getObject()->name : "NONE") << endl;
            result.put(img_name, guess);
        }
        // TODO: save experiment results
        CutSseResponseFunction response;
        response(result, testdesc, exp.response);

        getVcsCommit(exp.vcs_commit);
        
        ClutSegmenterStats stats = segmenter.getStats();
        exp.response.avg_keypoints = float(stats.keypoints) / stats.queries;
        exp.response.avg_detect_matches = float(stats.detect_matches) / stats.queries;
        exp.response.avg_detect_inliers = float(stats.detect_inliers) / stats.detect_guesses;
        exp.response.avg_detect_choice_matches = float(stats.detect_choice_matches) / stats.choices;
        exp.response.avg_detect_choice_inliers = float(stats.detect_choice_inliers) / stats.choices;
        exp.response.avg_locate_matches = float(stats.locate_matches) / stats.queries;
        exp.response.avg_locate_inliers = float(stats.locate_inliers) / stats.detect_guesses;
        exp.response.avg_locate_choice_matches = float(stats.locate_choice_matches) / stats.choices;
        exp.response.avg_locate_choice_inliers = float(stats.locate_choice_inliers) / stats.choices;

        time_t tt = time(NULL);
        tm *t = localtime(&tt);
        char ts[17];
        strftime(ts, 17, "%Y-%m-%d %H:%M:%S", t);
        exp.time = string(ts);
        exp.has_run = true;
    }

    void ExperimentRunner::run() {
        vector<Experiment> exps;
        while (true) {
            cout << "Querying database for experiments to carry out..." << endl;
            selectExperimentsNotRun(db_, exps);
            if (exps.empty()) {
                // Wait for someone inserting new rows into the database. The
                // experiment runner can run as a kind of daemon which
                // dispatches requests for new experiments.
                sleep(1);
                continue;
            }
            // Make it more likely that the training features do not have to be
            // reload. Since it is only a few seconds, it does not matter too
            // much, though.
            sortExperimentsByTrainFeatures(exps);
            TrainFeatures cur_tr_feat;
            ClutSegmenter *segmenter = NULL;
            BOOST_FOREACH(Experiment & exp, exps) {
                TrainFeatures tr_feat(exp.train_set, exp.paramset.train_pms_fe);
                if (tr_feat != cur_tr_feat) {
                    if (!cache_.trainFeaturesExist(tr_feat)) {
                        // TODO: catch error where train directory does not exist
                        // This is a critical part where the experiment runner
                        // should not be interrupted. Also proper closing of
                        // the database has to be ensured by a database handler
                        // FIXME:
                        tr_feat.generate();
                        cache_.addTrainFeatures(tr_feat);
                    }
                    delete segmenter;
                    segmenter = new ClutSegmenter(
                        cache_.trainFeaturesDir(tr_feat).string(),
                        TODParameters(), TODParameters());
                }

                // Clear statistics        
                segmenter->resetStats();

                // Online change configuration
                segmenter->reconfigure(exp.paramset);
                
                try {
                    runExperiment(*segmenter, exp);
                    exp.serialize(db_);
                } catch( runtime_error & e ) {
                    cerr << "[RUN]: " << e.what() << endl;
                    cerr << "[RUN]: ERROR, experiment failed, no results recorded (id=" << exp.id << ")" << endl;
                }
            }
            // TODO: use smart pointer
            if (segmenter != NULL) {
                delete segmenter;
            }
        }
    }

}

