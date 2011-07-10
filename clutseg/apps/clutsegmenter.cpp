 /*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/format.hpp>
    #include <boost/program_options.hpp>
    #include <pcl/io/pcd_io.h>
    #include <iostream>
    #include <stdlib.h>
    #include <string>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace std;
namespace po = boost::program_options;

// TODO: enable -Wunused-parameter for this file
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

struct Opts {
    // raw options
    string modelbase_dir;
    string query_image_file;
    string query_cloud_file;
    string camera_file;
    string inlier_cloud_file;
    string detect_config_file;
    string refine_config_file;
    bool visualize;
    // processed options
    Query query;
    Camera camera;
};

int options(int argc, char ** argv, Opts& opts) {
    // argument names
    string query_image("query_image");
    string modelbase("modelbase");

    po::options_description h("Hidden options");
    h.add_options()
        (modelbase.c_str(), po::value<string>(&opts.modelbase_dir), "Directory pointing to modelbase")
        (query_image.c_str(), po::value<string>(&opts.query_image_file), "Query image file");

    // default file names
    string def_detect_config_fn("detect.config.yaml");
    string def_refine_config_fn("refine.config.yaml");
    string def_camera_fn("camera.yaml");

    // option names
    string query_cloud("query_cloud");
    string detect_config("detect_config");
    string refine_config("refine_config");
    string camera("camera");

    po::options_description d("Allowed options");
    d.add_options()
        ("help", "Print this help message.")
        (query_cloud.c_str(),
            po::value<string>(&opts.query_cloud_file),
            "Query point cloud file")
        ("inlier_cloud",
            po::value<string>(&opts.inlier_cloud_file),
            "Destination point cloud file for inliers")
        (detect_config.c_str(),
            po::value<string>(&opts.detect_config_file),
            str(boost::format("Parameter set for detection. Default is <modelbase>/%s") % def_detect_config_fn).c_str())
        (refine_config.c_str(),
            po::value<string>(&opts.refine_config_file),
            str(boost::format("Parameter set for refinement. Default is <modelbase>/%s") % def_refine_config_fn).c_str())
        (camera.c_str(),
            po::value<string>(&opts.camera_file),
            str(boost::format("Camera info file. Default is <modelbase>/%s") % def_camera_fn).c_str())
        ("visualize",
            po::value<bool>(&opts.visualize)->zero_tokens(),
            "If specified, result will be displayed in a window");

    po::positional_options_description p;
    p.add(modelbase.c_str(), 1);
    p.add(query_image.c_str(), 1);

    po::options_description c;
    c.add(h).add(d);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(c).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help") || vm.count(modelbase) == 0 || vm.count(query_image) == 0)
    {
        cout << "usage: \033\[1mclutsegmenter\033[0m [options] " + modelbase + " " + query_image << endl;
        cout << endl;
        cout << d << endl;
        return 1;
    }

    // handle default values
    if (vm.count(detect_config) == 0) {
        opts.detect_config_file = opts.modelbase_dir + "/" + def_detect_config_fn;
    }

    if (vm.count(refine_config) == 0) {
        opts.refine_config_file = opts.modelbase_dir + "/" + def_refine_config_fn;
    }

    if (vm.count(camera) == 0) {
        opts.camera_file = opts.modelbase_dir + "/" + def_camera_fn;
    }

    // transform primitives into structs
    opts.query.img = imread(opts.query_image_file);
    if (vm.count(query_cloud) == 1) {
        io::loadPCDFile(opts.query_cloud_file, opts.query.cloud);
    }
    opts.camera = Camera(opts.camera_file, Camera::TOD_YAML);
    return 0;
}



int main(int argc, char **argv) {
    Opts opts;
    if (options(argc, argv, opts)) {
        return 1;
    }

    // Create segmenter
    Clutsegmenter segmenter(
        opts.modelbase_dir,
        opts.detect_config_file,
        opts.refine_config_file 
    );

    Result result;
    // Actual recognition 
    segmenter.recognize(opts.query, result);

    if (opts.visualize) {
        Mat c = opts.query.img.clone();
        drawGuess(c, result.refine_choice, opts.camera, PoseRT());

        imshow("Refined guess", c);
        waitKey(-1);

        /* This is a good idea, and quite promising for the tod 
        vector<tod::Guess> guesses;
        for (size_t i = 0; i < result.detect_choices.size(); i++) {
            if (result.detect_choices[i].inliers.size() > 50) {
                guesses.push_back(result.detect_choices[i]);
            }
        }
        drawGuesses(c, guesses, opts.camera);
        imshow("Detect choices", c);
        waitKey(-1);
        */
    }
    
    if (opts.inlier_cloud_file != "") {
        io::savePCDFileASCII(opts.inlier_cloud_file, result.refine_choice.inlierCloud);
    }

    return 0;
}

