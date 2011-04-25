 /*
 * Author: Julius Adorf
 */

#include "clutseg.h"

#include "pose_util.h"

#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace clutseg;
using namespace std;

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

void processOutput(int argc, char **argv, bool positive, Guess & guess, PointCloudT & inlierCloud) {
    // Process result
    if (positive) {
        cout << "Recognized " << guess.getObject()->name << endl;
        if (argc >= 4) {
            io::savePCDFileASCII(argv[3], inlierCloud);
        }
        if (argc >= 5) {
           writePose(argv[4], guess.aligned_pose()); 
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
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    // Input values
    Mat queryImage;
    PointCloudT queryCloud;

    readInput(argc, argv, queryImage, queryCloud);

    // Output values
    bool positive;
    Guess guess;
    PointCloudT inlierCloud;

    // Actual recognition 
    positive = segmenter.recognize(queryImage, queryCloud, guess, inlierCloud);

    processOutput(argc, argv, positive, guess, inlierCloud);

    return 0;
}

