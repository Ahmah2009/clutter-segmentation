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
        bfs::path exp_result_dir = result_dir_ / (str(boost::format("%05d") % experiment_id));
        if (!bfs::exists(exp_result_dir)) {
            bfs::create_directory(exp_result_dir);
            //throw runtime_error(str(boost::format(
            //    "ERROR: Result directory for experiment %d already exists.") % experiment_id));
        }

        // Draw locate choice image
        Mat locate_choice_img = img.clone();
        drawGroundTruth(locate_choice_img, ground, camera);
        if (result.guess_made) { // TODO: make locate_choice a method, then throw exception if !guess_made
            drawGuess(locate_choice_img, result.locate_choice, camera, PoseRT());
        }
        bfs::path locate_choice_img_path = exp_result_dir / (img_name + ".locate_choice.png");
        bfs::create_directories(locate_choice_img_path.parent_path());
        imwrite(locate_choice_img_path.string(), locate_choice_img);
        // Draw locate choice image
    }

}
