/**
 * Author: Julius Adorf
 */

#include "clutseg/runner.h"

#include "clutseg/check.h"
#include "clutseg/clutseg.h"
#include "clutseg/modelbase.h"
#include "clutseg/paramsel.h"
#include "clutseg/ranking.h"
#include "clutseg/response.h"
#include "clutseg/ground.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/date_time/posix_time/posix_time.hpp>
    #include <boost/foreach.hpp>
    #include <boost/thread.hpp>
    #include <ctime>
    #include <cv.h>
    #include <pcl/io/pcd_io.h>
    #include <string>
    #include <tod/detecting/Parameters.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

namespace clutseg {

    ExperimentRunner::ExperimentRunner() : terminate(false) {}

    ExperimentRunner::ExperimentRunner(sqlite3* db,
                                       const ModelbaseCache & cache,
                                       const ResultStorage & storage) :
                                        terminate(false), db_(db),
                                        cache_(cache), storage_(storage) {}

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

    // The test runner runs in two different levels. First, at the experiment
    // level, it iterates over different experiment setups.  Second, it
    // iterates over each test scene. We are doing this only, and really only
    // because we are interested in the statistics and data for later analysis.
    // Some data is local to each test scene.  Other data is global (or made
    // global by aggregation and averaging of locally gathered data. We have
    // two main tasks, (1) storing global results in the database, and (2)
    // storing local results to the filesystem. All information that is to be
    // stored in the database is coupled by a Response object (the name
    // 'response' is taken from Alpaydin and refers to the whole experiment),
    // and local data is written to the disk by a storage module. Basically,
    // both global and local processors often need the same data, such as angle
    // error and translational error, success or not, and so on.  Therefore,
    // the experiment runner generates a report for each test scene and passes
    // this to the global and local processors. There occurs the question,
    // whether the response object actually shall update itself according to
    // the report.  Also, how much information shall the Result object carry.
    // Actually, it should not carry any more information than the
    // Clutsegmenter can fill in. It can become a member of a report specific
    // to a test scene.
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    void ExperimentRunner::runExperiment(Clutsegmenter & sgm, Experiment & e) {
        bfs::path p = getenv("CLUTSEG_PATH");
        bfs::path test_dir = p / e.test_set;
        GroundTruth  testdesc = loadGroundTruth(test_dir / "ground-truth.txt");
        SetResult resultSet;
        bfs::path camera_path = test_dir / "camera.yml";
        assert_path_exists(camera_path);
        Camera camera(camera_path.string(), Camera::TOD_YAML);
        // http://www.gnu.org/s/libc/manual/html_mono/libc.html#CPU-Time
        float rt = 0;
        // Loop over all images in the test set
        for (GroundTruth::iterator test_it = testdesc.begin(); test_it != testdesc.end(); test_it++) {
            string img_name = test_it->first;
            bfs::path img_path = test_dir / img_name;
            Mat queryImage = imread(img_path.string());
            if (queryImage.empty()) {
                throw runtime_error(str(boost::format(
                    "ERROR: Cannot read image '%s' for eeriment with id=%d. Please check\n"
                    "whether image file exists. Full path is '%s'."
                ) % img_name % e.id % img_path));
            }
            cout << "[RUN] " << e.name << " - loaded image " << img_path << endl;
            PointCloudT queryCloud;
            bfs::path cloud_path = cloudPath(img_path);
            if (bfs::exists(cloud_path)) {
                pcl::io::loadPCDFile(cloud_path.string(), queryCloud);
                cout << "[RUN] Loaded query cloud " << cloud_path << endl;
            }
            Query query(queryImage, queryCloud);
            Result res;
            clock_t b = clock();
            sgm.recognize(query, res);
            rt += float(clock() - b) / CLOCKS_PER_SEC;
            cout << "[RUN] Recognized " << (res.guess_made ? res.refine_choice.getObject()->name : "NONE") << endl;
            resultSet[img_name] = res;
 
            TestReport report(e, query, res, test_it->second, img_name, test_dir, camera);
            storage_.record(report);

            if (terminate) {
                cout << "[RUN] Registered termination request. Program will be terminated as soon as the modelbase.has been carried out completely." << endl;
            }

            // The heavy load on both CPU and IO often makes the operating
            // system very unresponsive, so take a little break. This can
            // probably also achieved by adjusting 'nice' value.
            timespec t;
            t.tv_sec = 0;
            t.tv_nsec = int(1e8);
            nanosleep(&t, NULL);
        }

        CutSseResponseFunction responseFunc;
        responseFunc(resultSet, testdesc, sgm.getTemplateNames(), e.response);

        e.response.test_runtime = rt;

        sgm.getStats().populateResponse(e.response);
        e.record_time();
        e.record_commit();
        e.has_run = true;
    }

    void ExperimentRunner::skipExperimentsWhereFeatureExtractorCreateFailed(vector<Experiment> & exps) {
        // This is a preliminary check. Some feature configurations might be
        // invalid, or even some assertion failure might happen in
        // tod_training/src/feature_extraction.cpp Whatever the reason is and
        // whoever to blame, it's important to fail early such that not so much
        // time is wasted.
        BOOST_FOREACH(Experiment & e, exps) {
            if (terminate) {
                return;
            } else if (!e.skip && !(e.flags & Experiment::FLAG_FEPARAMS_VALID) && !(e.flags & Experiment::FLAG_FEPARAMS_INVALID)) {
                try {
                    cout << "[RUN]: Verifying that constructing a FeatureExtractor instance from supplied train features config works: " << e.name << endl;
                    FeatureExtractor::create(e.paramset.train_pms_fe);
                } catch (...) {
                    cerr << "[RUN]: ERROR, cannot construct FeatureExtractor instance from supplied test features config: " << e.name << endl;
                    e.machine_note = "Bad train_pms_fe, FeatureExtractor::create failed";
                    e.skip = true; 
                    e.flags |= Experiment::FLAG_FEPARAMS_INVALID;
                    e.serialize(db_);
                    continue;
                }
                try {
                    cout << "[RUN]: Verifying that constructing a FeatureExtractor instance from supplied test features config works: " << e.name << endl;
                    FeatureExtractor::create(e.paramset.recog_pms_fe);
                } catch (...) {
                    cerr << "[RUN]: ERROR, cannot construct FeatureExtractor instance from supplied test features config: " << e.name << endl;
                    e.machine_note = "Bad recog_pms_fe, FeatureExtractor::create failed";
                    e.skip = true; 
                    e.flags |= Experiment::FLAG_FEPARAMS_INVALID;
                    e.serialize(db_);
                    continue;
                }
                e.flags |= Experiment::FLAG_FEPARAMS_VALID;
            }
        }
    }

    void ExperimentRunner::skipExperimentsWhereNoFeaturesExtracted(vector<Experiment> & exps) {
        // Quickly verify whether the feature extraction params actually lead to the extraction
        // of any features. This quickly finds broken configurations that will never lead anywhere.
        Mat ver_img = imread("data/image_00000.png", 0);
        BOOST_FOREACH(Experiment & e, exps) {
            if (terminate) {
                return;
            } else if (!e.skip && !(e.flags & Experiment::FLAG_FEPARAMS_GOOD) && !(e.flags & Experiment::FLAG_FEPARAMS_BAD)) {
                cout << "[RUN]: Verifying that features are extracted when using train features config: " << e.name << endl;
                Ptr<FeatureExtractor> x = FeatureExtractor::create(e.paramset.train_pms_fe);
                Features2d xf;
                xf.image = ver_img.clone();
                x->detectAndExtract(xf);
                if (xf.keypoints.size() == 0) {
                    cerr << "[RUN]: " << e.name << " - ERROR, no features extracted when using train features config: " << e.name << endl;
                    e.machine_note = "Bad train_pms_fe, no features extracted";
                    e.skip = true; 
                    e.flags |= Experiment::FLAG_FEPARAMS_BAD;
                    e.serialize(db_);
                    continue;
                } else {
                    cout << "[RUN]: " << xf.keypoints.size() << " keypoints extracted on a validation image using train_pms_fe of " << e.name << endl;
                }
                Ptr<FeatureExtractor> y = FeatureExtractor::create(e.paramset.recog_pms_fe);
                Features2d yf;
                yf.image = ver_img.clone();
                y->detectAndExtract(yf);
                if (yf.keypoints.size() == 0) {
                    cout << "[RUN]: ERROR, no features extracted when using test features config: " << e.name << endl;
                    e.machine_note = "Bad recog_pms_fe, no features extracted";
                    e.skip = true; 
                    e.flags |= Experiment::FLAG_FEPARAMS_BAD;
                    e.serialize(db_);
                    continue;
                } else {
                    cout << "[RUN]: " << yf.keypoints.size() << " keypoints extracted on a validation image using recog_pms_fe of " << e.name << endl;
                }
                e.flags |= Experiment::FLAG_FEPARAMS_GOOD;
            }
        }
    }

    void generate(Modelbase & tr_feat) {
        tr_feat.generate();
    }

    void ExperimentRunner::run() {
        while (!terminate) {
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
            sortExperimentsByModelbase(exps);
            Modelbase cur_tr_feat;
            Clutsegmenter *sgm = NULL;

            skipExperimentsWhereFeatureExtractorCreateFailed(exps);
            if (terminate) {
                return;
            }
            skipExperimentsWhereNoFeaturesExtracted(exps);
            if (terminate) {
                return;
            }

            BOOST_FOREACH(Experiment & e, exps) {
                if (terminate) {
                    break;
                } else if (e.skip) {
                    cerr << "[RUN]: Skipping experiment (id=" << e.id << ")" << endl;
                } else {
                    Modelbase tr_feat(e.train_set, e.paramset.train_pms_fe);
                    if (tr_feat != cur_tr_feat) {
                        if (!cache_.modelbaseExist(tr_feat)) {
                            int max_seconds = 2400;
                            if (cache_.modelbaseBlacklisted(tr_feat)) {
                                e.skip = true;
                                e.flags |= Experiment::FLAG_TRAIN_TIMEOUT;
                                e.machine_note = str(boost::format("took longer than %d for training") % max_seconds);
                                e.serialize(db_);
                                continue;
                            }
                            // This is a critical part where the experiment runner
                            // should not be interrupted. Also proper closing of
                            // the database has to be ensured by a database handler
                            boost::thread g(generate, tr_feat);
                            // 10 seconds per picture max, 4*60 = 240 pictures in total
                            // so maximum 2400 seconds = 40 minutes.
                            if (g.timed_join(boost::posix_time::seconds(max_seconds))) {
                                if (terminate) break;
                                cerr << "[RUN]: Adding training features to cache " << e.name << endl;
                                cache_.addModelbase(tr_feat);
                            } else {
                                cache_.blacklistModelbase(tr_feat);
                                e.skip = true;
                                e.flags |= Experiment::FLAG_TRAIN_TIMEOUT;
                                e.machine_note = str(boost::format("took longer than %d for training") % max_seconds);
                                e.serialize(db_);
                                g.interrupt();
                                g.join();
                                cerr << "[RUN]: ERROR, took more than " << max_seconds << " seconds for training: " << e.name << endl;
                                continue;
                            }
                        }
                        delete sgm;
                        sgm = new Clutsegmenter(
                            cache_.modelbaseDir(tr_feat).string(),
                            TODParameters(), TODParameters());
                    }

                    e.response.train_runtime = cache_.trainRuntime(tr_feat);                    

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

