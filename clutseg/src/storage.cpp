/**
 * Author: Julius Adorf
 */

#include "clutseg/storage.h"

#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <iostream>
    #include <opencv2/highgui/highgui.hpp>
    #include <tod/core/Features2d.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;
namespace bfs = boost::filesystem;

namespace clutseg {

    void store_config(const bfs::path & filename, const TODParameters & params) {
        bfs::create_directories(filename.parent_path());
        FileStorage fs(filename.string(), FileStorage::WRITE);
        fs << TODParameters::YAML_NODE_NAME;
        params.write(fs);
        fs.release();
    }

    string cut_file_extension(const string & filename) {
        size_t offs = filename.rfind(".");
        string img_basename = filename;
        if (offs == string::npos) {
            return filename;
        } else {
            return img_basename.substr(0, offs);
        }
    }

    void ResultStorage::record(const TestReport & report) {
        bfs::path erd = result_dir_ / (str(boost::format("%05d") % report.experiment.id));
        if (!bfs::exists(erd)) {
            bfs::create_directory(erd);
        }

        cout << boost::format("[STORE] Saving result on '%s' for experiment '%d'") % report.img_name % report.experiment.id << endl;

        string img_basename = cut_file_extension(report.img_name);

        // Draw locate choice image
        Mat lci = report.query.img.clone();
        drawGroundTruth(lci, report.ground, report.camera);
        if (report.result.guess_made && report.ground.onScene(report.result.refine_choice.getObject()->name)) { 
            drawGuess(lci, report.result.refine_choice, report.camera, PoseRT());
            vector<string> err_text;
            err_text.push_back(str(boost::format("angle_error: %4.2f deg") % (report.angle_error() * 360 / (2 * M_PI))));
            err_text.push_back(str(boost::format("trans_error: %4.2f cm") % (report.trans_error() * 100)));
            drawText(lci, err_text, Point(20, 80), CV_FONT_HERSHEY_SIMPLEX, 0.7, 1, Scalar(255, 255, 255));
        }
        if (report.success()) {
            vector<string> succ_text(1, "SUCCESS");
            drawText(lci, succ_text, Point(10, 10), CV_FONT_HERSHEY_SIMPLEX, 1.6, 3, Scalar(0, 204, 0));
        } else {
            vector<string> fail_text(1, "FAILURE");
            drawText(lci, fail_text, Point(10, 10), CV_FONT_HERSHEY_SIMPLEX, 1.6, 3, Scalar(0, 0, 255));
        }

        bfs::path lci_path = erd / (img_basename + ".refine_choice.png");
        bfs::create_directories(lci_path.parent_path());
        imwrite(lci_path.string(), lci);

        // Draw detect choices image
        Mat dci = report.query.img.clone();
        drawGroundTruth(dci, report.ground, report.camera);
        drawGuesses(dci, report.result.detect_choices, report.camera);
        bfs::path dci_path = erd / (img_basename + ".detect_choices.png");
        bfs::create_directories(dci_path.parent_path());
        imwrite(dci_path.string(), dci);

        // Save keypoints
        // TODO: extract method
        bfs::path feat_path = erd / (img_basename + ".features.yaml.gz");
        bfs::create_directories(feat_path.parent_path());
        FileStorage feat_fs(feat_path.string(), FileStorage::WRITE);
        feat_fs << Features2d::YAML_NODE_NAME;
        report.result.features.write(feat_fs);
        feat_fs.release();

        // Save locate choice
        // TODO: extract method
        bfs::path lc_path = erd / (img_basename + ".refine_choice.yaml.gz");
        bfs::create_directories(lc_path.parent_path());
        LabelSet lls;
        if (report.result.guess_made) {
            lls.labels.push_back(Label(
                report.result.refine_choice.getObject()->name,
                poseToPoseRT(report.result.refine_choice.aligned_pose())));
        }
        writeLabelSet(lc_path, lls);

        // Save detect choices 
        bfs::path dc_path = erd / (img_basename + ".detect_choices.yaml.gz");
        bfs::create_directories(dc_path.parent_path());
        // TODO: extract convert method
        LabelSet dls;
        BOOST_FOREACH(const Guess & c, report.result.detect_choices) {
            dls.labels.push_back(Label(c.getObject()->name, poseToPoseRT(c.aligned_pose())));
        }
        writeLabelSet(dc_path, dls);

        TODParameters dp = report.experiment.paramset.toDetectTodParameters();
        store_config(erd / "detect.config.yaml", dp);

        TODParameters lp = report.experiment.paramset.toLocateTodParameters();
        store_config(erd / "locate.config.yaml", lp);
    }

}
