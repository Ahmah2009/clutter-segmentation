/*
 * Author: Julius Adorf
 *
 * Shows keypoints for a certain training view.
 */

#include <tod/core/Features2d.h>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv.h>

using namespace std;
using namespace cv;
using namespace tod;
using namespace boost;

int main(int argc, char **argv) {
    if (argc != 4 && argc != 5) {
        cerr << "Usage: keypoints_viewer <base> <subject> <view-number> [<outfile>]" << endl;
        return -1;
    }
    // string base = "/home/julius/Studium/BA/tod_kinect_train_15/fat_free_milk/";
    // int view = 0;
    string base(argv[1]);
    string subject(argv[2]);
    int view = lexical_cast<int>(argv[3]);
    string outfile("");
    if (argc == 5) {
        outfile = argv[4];
    }

    string imgp = str(boost::format("%s/%s/image_%05d.png") % base % subject % view);
    string f2dp = imgp + ".features.yaml.gz";

    cout << "base: " << base << endl;
    cout << "subject: " << subject << endl;
    cout << "view-number: " << view << endl;
    cout << "image path: " << imgp << endl;
    cout << "f2d path: " << f2dp << endl;

    // load image
    Mat img = imread(imgp, 0); 
    Mat canvas = img.clone();

    // load keypoints 
    FileStorage fs(f2dp, FileStorage::READ);
    Features2d f2d;
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = img;

    // draw image and keypoints
    f2d.draw(canvas, 0);
    if (outfile != "") {
        imwrite(outfile, canvas);
    }
    imshow("features", canvas);
    waitKey(-1);
    return 0;
}

