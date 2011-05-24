 /*
 * Author: Julius Adorf
 */

#include "clutseg/clutseg.h"

#include "clutseg/common.h"
#include "clutseg/pose.h"

#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace clutseg;

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
        cout << "Recognized " << result.locate_choice.getObject()->name << endl;
        if (argc >= 4) {
            io::savePCDFileASCII(argv[3], result.locate_choice.inlierCloud);
        }
        if (argc >= 5) {
           writePose(argv[4], result.locate_choice.aligned_pose()); 
        }
    } else {
        cerr << "Could not recognize object." << endl;
        abort();
    }
}


int main(int argc, char **argv) {
    // Create segmenter
    ClutSegmenter segmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    // Input: test image
    Mat queryImage;
    // Input: corresponding point cloud
    PointCloudT queryCloud;

    ClutsegQuery query(queryImage, queryCloud);

    readInput(argc, argv, queryImage, queryCloud);

    Result result;
    // Actual recognition 
    segmenter.recognize(query, result);

    processOutput(argc, argv, result);

    return 0;
}

