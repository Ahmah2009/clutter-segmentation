/*
 * Author: Julius Adorf
 */

#include <stdlib.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>
#include <cv.h>

#include "clutseg/pcl_visualization_addons.h"
#include "clutseg/pose_util.h"

using namespace std;
using namespace pcl;
using namespace pcl_visualization;
using namespace cv;
using namespace opencv_candidate;
using namespace boost;
using namespace clutseg;

void randomize(string path, double stddev_t, double stddev_r, bool serialize, bool visualize) {
        FileStorage in(path, FileStorage::READ);
        PoseRT pose;
        pose.read(in[PoseRT::YAML_NODE_NAME]);
        in.release();

        PoseRT rpose;
        rpose.tvec = pose.tvec.clone();
        rpose.rvec = pose.rvec.clone();
        randomizePose(rpose, stddev_t, stddev_r);
        cout << "tvec[0] --- original: " << pose.tvec.at<double>(0, 0) << ", randomized: " << rpose.tvec.at<double>(0, 0) << endl;
        cout << "tvec[1] --- original: " << pose.tvec.at<double>(1, 0) << ", randomized: " << rpose.tvec.at<double>(1, 0) << endl;
        cout << "tvec[2] --- original: " << pose.tvec.at<double>(2, 0) << ", randomized: " << rpose.tvec.at<double>(2, 0) << endl;
        cout << "rvec[0] --- original: " << pose.rvec.at<double>(0, 0) << ", randomized: " << rpose.rvec.at<double>(0, 0) << endl;
        cout << "rvec[1] --- original: " << pose.rvec.at<double>(1, 0) << ", randomized: " << rpose.rvec.at<double>(1, 0) << endl;
        cout << "rvec[2] --- original: " << pose.rvec.at<double>(2, 0) << ", randomized: " << rpose.rvec.at<double>(2, 0) << endl;
        
        if (visualize) { 
            PCLVisualizer visualizer;
            visualizer.addCoordinateSystem(0.5, 0, 0, 0);
            addPose(visualizer, pose, "pose");           
            addPose(visualizer, rpose, "rpose");
            visualizer.spin();
        }
        
        if (serialize) {
            cout << "serializing!" << endl;
            FileStorage out(path, FileStorage::WRITE);
            out << PoseRT::YAML_NODE_NAME;
            rpose.write(out);
            out.release();
        }
}

int main(int argc, char **argv) {
    // Read input arguments
    if (argc != 6) {
        cout << "Usage: poserandomizer (<pose-directory>|<pose-file>) <stddev-translation> <stddev-rotation> <serialize> <visualize>" << endl;
        return 1;
    }
    string path(argv[1]);
    double stddev_t = boost::lexical_cast<double>(argv[2]);
    double stddev_r = boost::lexical_cast<double>(argv[3]);
    bool serialize = (1 == boost::lexical_cast<int>(argv[4]));
    bool visualize = (1 == boost::lexical_cast<int>(argv[5]));

    if (boost::filesystem::is_directory(path)) {
        boost::filesystem::directory_iterator it(path);
        boost::filesystem::directory_iterator end;
        while (it != end) {
            string fname = it->filename();
            if (ends_with(fname, ".pose.yaml")) {
                cout << fname << endl;
                randomize(it->path().string(), stddev_t, stddev_r, serialize, visualize);
            }
            it++;
        }
    } else {
        randomize(path, stddev_t, stddev_r, serialize, visualize);
    }
}

