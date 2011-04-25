 /*
 * Author: Julius Adorf
 */

#include "clutseg.h"

#include "pose_util.h"

#include <pcl/io/pcd_io.h>
#include <tod/detecting/Loader.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace clutseg;
using namespace std;

void loadParams(ClutSegmenter & segmenter) {
    FileStorage fs(segmenter.opts.config, FileStorage::READ);
    if (!fs.isOpened()) {
        throw ios_base::failure("Cannot read configuration file '" + segmenter.opts.config + "'");
    }
    segmenter.opts.params.read(fs[tod::TODParameters::YAML_NODE_NAME]);
    fs.release();
}

void loadBase(ClutSegmenter & segmenter) {
    tod::Loader loader(segmenter.opts.baseDirectory);
    vector<Ptr<TexturedObject> > objects;
    loader.readTexturedObjects(objects);
    segmenter.base = TrainingBase(objects);
}

void createSegmenter(const string & baseDirectory, const string & config, ClutSegmenter & segmenter) {
    segmenter.opts.config = config;
    segmenter.opts.baseDirectory = baseDirectory;
    loadParams(segmenter);
    loadBase(segmenter);
}

int main(int argc, char **argv) {
    // Creating segmenter
    ClutSegmenter segmenter;
    createSegmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml", segmenter);

    // Input values
    Mat queryImage;
    PointCloudT queryCloud;

    // Output values
    bool positive;
    Guess guess;
    PointCloudT inlierCloud;

    // Read input
    if (argc <= 2) {
        cerr << "Usage: clutsegmenter <query-image> <query-cloud> [<inlier-cloud-out>] [<pose-out>]" << endl;
        return 1;
    }
    queryImage = imread(argv[1]);
    io::loadPCDFile(argv[2], queryCloud);
    if (queryImage.empty()) {
        cerr << "Cannot read query image." << endl;
        return 1;
    }

    positive = segmenter.recognize(queryImage, queryCloud, guess, inlierCloud);

    if (positive) {
        cout << "Recognized " << guess.getObject()->name << endl;
        if (argc >= 4) {
            io::savePCDFileASCII(argv[3], inlierCloud);
        }
        if (argc >= 5) {
           writePose(argv[4], guess.aligned_pose()); 
        }
        return 0;
    } else {
        cerr << "Could not recognize object." << endl;
        return 1;
    }
}
