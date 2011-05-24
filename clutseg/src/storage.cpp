/**
 * Author: Julius Adorf
 */

#include "clutseg/storage.h"

#include "clutseg/viz.h"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace cv;
using namespace opencv_candidate;
using namespace std;
namespace bfs = boost::filesystem;

namespace clutseg {

    /** Stores results for one test scene. */
    void ResultStorage::store(int experiment_id,
                const bfs::path & test_dir,
                const string & img_name,
                const Mat & img,
                const Camera & camera,
                const GroundTruth & ground,
                const Result & result) {
        bfs::path erd = result_dir_ / (str(boost::format("%05d") % experiment_id));
        if (!bfs::exists(erd)) {
            bfs::create_directory(erd);
            //throw runtime_error(str(boost::format(
            //    "ERROR: Result directory for experiment %d already exists.") % experiment_id));
        }

        // Draw locate choice image
        Mat lci = img.clone();
        drawGroundTruth(lci, ground, camera);
        if (result.guess_made) { // TODO: make locate_choice a method, then throw exception if !guess_made
            drawGuess(lci, result.locate_choice, camera, PoseRT());
        }
        bfs::path lci_path = erd / (img_name + ".locate_choice.png");
        bfs::create_directories(lci_path.parent_path());
        imwrite(lci_path.string(), lci);

        // Draw detect choices image
        Mat dci = img.clone();
        drawGroundTruth(dci, ground, camera);
        vector<PoseRT> dummy;
        drawGuesses(dci, result.detect_choices, camera, dummy); // TODO: create delegate method or use default parameter
        bfs::path dci_path = erd / (img_name + ".detect_choices.png");
        bfs::create_directories(dci_path.parent_path());
        imwrite(dci_path.string(), dci);
    }

}
