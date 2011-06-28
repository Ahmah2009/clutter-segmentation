 /*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <pcl/io/pcd_io.h>
    #include <stdlib.h>
    #include <iostream>
    #include <string>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace clutseg;

// TODO: enable -Wunused-parameter for this file
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

void readInput(int argc, char **argv, Mat & queryImage, PointCloudT & queryCloud) {
    // Read input
    if (argc <= 2) {
        cerr << "Usage: clutsegmenter <query-image> <query-cloud> [<inlier-cloud-out>] [<pose-out>]" << endl;
        abort();
    }
    queryImage = imread(argv[1]);
    io::loadPCDFile(argv[2], queryCloud);
    if (queryImage.empty()) {
        cerr << "Cannot read query image." << endl;
        abort();
    }
}

void processOutput(int argc, char **argv, Result & result) {
    // Process result
    if (result.guess_made) {
        cout << "Recognized " << result.refine_choice.getObject()->name << endl;
        if (argc >= 4) {
            io::savePCDFileASCII(argv[3], result.refine_choice.inlierCloud);
        }
        if (argc >= 5) {
           writePose(argv[4], result.refine_choice.aligned_pose()); 
        }
    } else {
        cerr << "Could not recognize object." << endl;
        abort();
    }
}


int main(int argc, char **argv) {
    // Create segmenter
    Clutsegmenter segmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    Query query;

    readInput(argc, argv, query.img, query.cloud);

    Result result;
    // Actual recognition 
    segmenter.recognize(query, result);

    processOutput(argc, argv, result);

    return 0;
}

