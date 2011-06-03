/*
 * Author: Julius Adorf
 */

#include "clutseg/conn_comp.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <iostream>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << "Usage: fixmask <src-image> <dst-image>" << endl;
        return 1;
    }
    Mat mask = imread(argv[1], 0);
    mask = mask > 1;
    largestConnectedComponent(mask);
    imwrite(argv[2], mask);
    return 0;
}

