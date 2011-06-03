/*
 * Author: Julius Adorf
 *
 * Shows keypoints for a certain training view.
 */

#include "clutseg/check.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <boost/format.hpp>
    #include <boost/lexical_cast.hpp>
    #include <cv.h>
    #include <iostream>
    #include <opencv2/highgui/highgui.hpp>
    #include <tod/core/Features2d.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace clutseg;
using namespace cv;
using namespace tod;
using namespace boost;

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
    if (argc != 4 && argc != 5) {
        cerr << "Usage: keypoints_viewer <base> <subject> <view-number> [<outfile>]" << endl;
        return -1;
    }
    string base(argv[1]);
    string subject(argv[2]);
    int view = lexical_cast<int>(argv[3]);
    bfs::path outfile("");
    if (argc == 5) {
        outfile = bfs::path(argv[4]);
    }

    bfs::path imgp = str(boost::format("%s/%s/image_%05d.png") % base % subject % view);
    bfs::path f2dp = bfs::path(imgp.string() + ".features.yaml.gz");

    assert_path_exists(imgp);
    assert_path_exists(f2dp);

    cout << "base: " << base << endl;
    cout << "subject: " << subject << endl;
    cout << "view-number: " << view << endl;
    cout << "image path: " << imgp << endl;
    cout << "f2d path: " << f2dp << endl;

    // load image
    Mat gray = imread(imgp.string(), 0); 
    Mat canvas = imread(imgp.string());

    // load keypoints 
    FileStorage fs(f2dp.string(), FileStorage::READ);
    Features2d f2d;
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = gray;

    // draw image and keypoints
    //f2d.draw(canvas, 0);
    drawKeypoints(f2d.image, f2d.keypoints, canvas, Scalar(0, 0, 255)); 
    if (outfile.string() != "") {
        imwrite(outfile.string(), canvas);
    }
    imshow("features", canvas);
    waitKey(-1);
    return 0;
}

