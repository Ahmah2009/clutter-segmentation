/**
 * Author: Julius Adorf
 */

#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include <boost/filesystem.hpp>
#include <cv.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_candidate/Camera.h>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace opencv_candidate;

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
    if (argc != 5) {
        cerr << "Usage: draw_ground_truth <src> <ground> <camera> <dst>" << endl;
        return -1;
    }

    bfs::path src = argv[1];
    bfs::path gtr = argv[2];
    bfs::path cam = argv[3];
    bfs::path dst = argv[4];

    Mat img = imread(src.string());

    LabelSet ls;
    readLabelSet(gtr, ls);

    Camera camera(cam.string(), Camera::TOD_YAML);

    Mat c = img.clone();
    drawGroundTruth(c, ls, camera);

    imwrite(dst.string(), c);
}
