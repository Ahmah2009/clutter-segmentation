/**
 * Author: Julius Adorf
 */

#include "clutseg/runner.h"

#include "clutseg/clutseg.h"
#include "clutseg/experiment.h"
#include "clutseg/paramsel.h"
#include "clutseg/ranking.h"
#include "clutseg/testdesc.h"

#include <boost/foreach.hpp>
#include <cv.h>
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

    void ExperimentRunner::runExperiment(const ClutSegmenter & segmenter, Experiment & exp) {
        // TODO: check which statistics might be useful and cannot be generated afterwards
        bfs::path p = getenv("CLUTSEG_PATH");
        bfs::path test_dir = p / exp.test_set;
        TestDesc testdesc = loadTestDesc(test_dir / "testdesc.txt");
        // Loop over all images in the test set
        for (TestDesc::iterator it = testdesc.begin(); it != testdesc.end(); it++) {
            string img_name = it->first;
            bfs::path img_path = test_dir / img_name;
            Features2d query;
            query.image = imread(img_path.string(), 0);
            if (query.image.empty()) {
                throw runtime_error(str(boost::format(
                    "Cannot read image '%s' for experiment with id=%d. Please check\n"
                    "whether image file exists. Full path is '%s'."
                ) % img_name % exp.id % img_path));
            }
            PointCloudT queryCloud;
            // TODO: this is a hack, cloud filenames are magically extracted
            bfs::path cloud_path = cloudPath(img_path);
        }
    }

    void ExperimentRunner::run() {
        vector<Experiment> exps;
        while (true) {
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
            ClutSegmenter segmenter;
            BOOST_FOREACH(Experiment & exp, exps) {
                TrainFeatures tr_feat(exp.train_set, exp.paramset.train_pms_fe);
                if (tr_feat != cur_tr_feat) {
                    if (!cache_.trainFeaturesExist(tr_feat)) {
                        // This is a critical part where the experiment runner
                        // should not be interrupted. Also proper closing of
                        // the database has to be ensured by a database handler
                        // FIXME:
                        tr_feat.generate();
                        cache_.addTrainFeatures(tr_feat);
                    }
                    segmenter = ClutSegmenter(
                        cache_.trainFeaturesDir(tr_feat).string(),
                        TODParameters(), TODParameters());
                }

                // Online change configuration
                segmenter.reconfigure(exp.paramset);
                
                try {
                    runExperiment(segmenter, exp);
                    // populate the response and results
                    exp.serialize(db_);
                } catch( runtime_error & e ) {
                    cerr << "[RUN]: " << e.what() << endl;
                    cerr << "[RUN]: ERROR, experiment failed, no results recorded (id=" << exp.id << ")" << endl;
                }
            }
        }
    }

}

