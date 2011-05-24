/**
 * Author: Julius Adorf
 */

#include "clutseg/runner.h"

#include "clutseg/check.h"
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
using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

namespace clutseg {

    ExperimentRunner::ExperimentRunner() {}

    ExperimentRunner::ExperimentRunner(sqlite3* db,
                                       const TrainFeaturesCache & cache,
                                       const ResultStorage & storage) :
                                        db_(db), cache_(cache), storage_(storage) {}

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

    // The test runner runs in two different levels. First, at the
    // experiment level, it iterates over different experiment setups.
    // Second, it iterates over each test scene. We are doing this only,
    // and really only because we are interested in the statistics and
    // data for later analysis. Some data is local to each test scene.
    // Other data is global (or made global by aggregation and averaging
    // of locally gathered data. We have two main tasks, (1) storing
    // global results in the database, and (2) storing local results to
    // the filesystem. All information that is to be stored in the database is coupled
    // by a Response object (the name 'response' is taken from Alpaydin and refers
    // to the whole experiment), and local data is written to the disk by a storage
    // module. Basically, both global and local processors often need the same
    // data, such as angle error and translational error, success or not, and so on.
    // Therefore, the experiment runner generates a report for each test scene and
    // passes this to the global and local processors. There occurs the question, whether
    // the response object actually shall update itself according to the report.
    // Also, how much information shall the Result object carry. Actually, it should not
    // carry any more information than the ClutSegmenter can fill in. It can become a member
    // of a report specific to a test scene.
    void ExperimentRunner::runExperiment(ClutSegmenter & sgm, Experiment & e) {
        bfs::path p = getenv("CLUTSEG_PATH");
        bfs::path test_dir = p / e.test_set;
        SetGroundTruth testdesc = loadSetGroundTruth(test_dir / "testdesc.txt");
        SetResult resultSet;
        bfs::path camera_path = test_dir / "camera.yml";
        assert_path_exists(camera_path);
        Camera camera(camera_path.string(), Camera::TOD_YAML);
        // Loop over all images in the test set
        for (SetGroundTruth::iterator test_it = testdesc.begin(); test_it != testdesc.end(); test_it++) {
            string img_name = test_it->first;
            bfs::path img_path = test_dir / img_name;
            Mat queryImage = imread(img_path.string());
            if (queryImage.empty()) {
                throw runtime_error(str(boost::format(
                    "ERROR: Cannot read image '%s' for eeriment with id=%d. Please check\n"
                    "whether image file exists. Full path is '%s'."
                ) % img_name % e.id % img_path));
            }
            cout << "[RUN] Loaded image " << img_path << endl;
            PointCloudT queryCloud;
            bfs::path cloud_path = cloudPath(img_path);
            if (bfs::exists(cloud_path)) {
                pcl::io::loadPCDFile(cloud_path.string(), queryCloud);
                cout << "[RUN] Loaded query cloud " << cloud_path << endl;
            }
            ClutsegQuery query(queryImage, queryCloud);
            Result res;
            bool pos = sgm.recognize(query, res);
            cout << "[RUN] Recognized " << (pos ? res.locate_choice.getObject()->name : "NONE") << endl;
            resultSet[img_name] = res;
 
            storage_.store(e.id, test_dir, img_name, queryImage,
                            camera, test_it->second, res); 
        }
        // TODO: save experiment results
        CutSseResponseFunction responseFunc;
        responseFunc(resultSet, testdesc, sgm.getTemplateNames(), e.response);

        sgm.getStats().populateResponse(e.response);
        e.record_time();
        e.record_commit();
        e.has_run = true;
    }

    void ExperimentRunner::run() {
        while (true) {
            cout << "[RUN] Querying database for experiments to carry out..." << endl;
            vector<Experiment> exps;
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
            ClutSegmenter *sgm = NULL;
            BOOST_FOREACH(Experiment & e, exps) {
                if (e.skip) {
                    cerr << "[RUN]: Skipping experiment (id=" << e.id << ")" << endl;
                } else {
                    TrainFeatures tr_feat(e.train_set, e.paramset.train_pms_fe);
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
                        delete sgm;
                        sgm = new ClutSegmenter(
                            cache_.trainFeaturesDir(tr_feat).string(),
                            TODParameters(), TODParameters());
                    }

                    // Clear statistics        
                    sgm->resetStats();

                    // Online change configuration
                    sgm->reconfigure(e.paramset);
                    
                    try {
                        runExperiment(*sgm, e);
                        e.serialize(db_);
                    } catch( runtime_error & err ) {
                        cerr << "[RUN]: " << err.what() << endl;
                        cerr << "[RUN]: ERROR, experiment failed, no results recorded (id=" << e.id << ")" << endl;
                        cerr << "[RUN]: Before running the experiment again, make sure to clear 'skip' flag in experiment record." << endl;
                        e.skip = true;
                        e.serialize(db_);
                    }
                }
            }
            if (sgm != NULL) {
                delete sgm;
            }

            sleep(3);
        }
    }

}

