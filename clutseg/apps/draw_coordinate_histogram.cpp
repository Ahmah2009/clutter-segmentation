/**
 * Author: Julius Adorf
 */

#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <boost/program_options.hpp>
    #include <cv.h>
    #include <iostream>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl_ros/transforms.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/filters/radius_outlier_removal.h>
    #include <tod/core/TexturedObject.h>
    #include <tod/core/TrainingBase.h>
    #include <tod/detecting/Loader.h>
    #include <tod/detecting/Tools.h>
    // Put this include at the very end, otherwise the compiler will
    // complain with lengthy messsages
    #include <opencv2/core/eigen.hpp>
    #include <opencv2/highgui/highgui.hpp>
#include "clutseg/gcc_diagnostic_enable.h"


using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace std;
namespace po = boost::program_options;

struct Opts {
    float xmin;
    float xmax;
    float xw;
    float ymin;
    float ymax;
    float yw;
    string config;
    string hist_file;
    string model_cloud_file;
};

int options(int argc, char **argv, Opts & opts) {
    string model_cloud = "model_cloud";
    string hist = "hist";

    po::options_description h("Hidden options");
    h.add_options()
        (model_cloud.c_str(), po::value<string>(&opts.model_cloud_file), "model cloud (see assemble_model)")
        (hist.c_str(), po::value<string>(&opts.hist_file), "destination image (histogram)");

    po::options_description d("Allowed options");
    d.add_options()
        ("xmin", po::value<float>(&opts.xmin)->default_value(-0.04f), "minimum x-value included")
        ("xmax", po::value<float>(&opts.xmax)->default_value(0.12f), "maximum x-value included")
        ("xw", po::value<float>(&opts.xw)->default_value(0.0005f), "bin width in direction x")
        ("ymin", po::value<float>(&opts.ymin)->default_value(-0.04f), "minimum y-value included")
        ("ymax", po::value<float>(&opts.ymax)->default_value(0.12f), "maximum y-value included")
        ("yw", po::value<float>(&opts.yw)->default_value(0.0005f), "bin width in direction y")
        ("config", po::value<string>(&opts.config)->default_value("ias"), "use pre-specified parameters (config=ias|tod)");

    po::positional_options_description p;
    p.add(model_cloud.c_str(), 1);
    p.add(hist.c_str(), 1);
 
    po::options_description c;
    c.add(d).add(h);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(c).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help") != 0
            || vm.count(model_cloud) == 0
            || vm.count(hist) == 0
            || !(opts.config == "tod" || opts.config == "ias")) {
        cout << "usage: \033\[1mdraw_coordinate_histogram\033[0m " << model_cloud << " " << hist << endl;
        cout << endl;
        cout << d;
        return 1;
    }

    if (opts.config == "tod") {
        opts.xmin = -0.07f;
        opts.xmax = 0.09f;
        opts.xw = 0.0005f;
        opts.ymin = -0.07f;
        opts.ymax = 0.09f;
        opts.yw = 0.0005f;
    }

    return 0;
}

int main(int argc, char **argv) {
    Opts opts;
    if (options(argc, argv, opts)) {
        return 1;
    }

    PointCloud<PointXYZ> model;
    io::loadPCDFile(opts.model_cloud_file, model);

    Mat xy_hist_bgr;
    drawCoordinateHist(xy_hist_bgr, model, XY,
            opts.xmin, opts.xmax, opts.xw,
            opts.ymin, opts.ymax, opts.yw, true);
    imwrite(opts.hist_file, xy_hist_bgr);

    return 0;
}

