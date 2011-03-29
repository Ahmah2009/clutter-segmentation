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

#include "pcl_visualization_addons.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace boost;
using namespace boost::filesystem;
using namespace boost::algorithm;

void randomize(PoseRT & pose, double stddev_t, double stddev_r) {
    mt19937 twister; 
    normal_distribution<> n_t(0, stddev_t);
    variate_generator<mt19937&, normal_distribution<> > noise_t(twister, n_t);
    pose.tvec.at<double>(0, 0) += noise_t();
    pose.tvec.at<double>(1, 0) += noise_t();
    pose.tvec.at<double>(2, 0) += noise_t();
    normal_distribution<> n_r(0, stddev_r);
    variate_generator<mt19937&, normal_distribution<> > noise_r(twister, n_r);
    pose.rvec.at<double>(0, 0) += noise_r();
    pose.rvec.at<double>(1, 0) += noise_r();
    pose.rvec.at<double>(2, 0) += noise_r();
}

int main(int argc, char **argv) {
    // Read input arguments
    cout << "Usage: poserandomizer <pose-directory> <stddev-translation> <stddev-rotation> <serialize> <visualize>" << endl;
    string dir(argv[1]);
    double stddev_t = boost::lexical_cast<double>(argv[2]);
    double stddev_r = boost::lexical_cast<double>(argv[3]);
    bool serialize = boost::lexical_cast<bool>(argv[4]);
    bool visualize = boost::lexical_cast<bool>(argv[4]);

    boost::filesystem::directory_iterator it(dir);
    boost::filesystem::directory_iterator end;
    while (it != end) {
        string fname = it->filename();
        if (ends_with(fname, ".pose.yaml")) {
            cout << fname << endl;
            FileStorage in(it->path().string(), FileStorage::READ);
            PoseRT pose;
            pose.read(in[PoseRT::YAML_NODE_NAME]);
    // TODO: do you have to close file storage before opening for writing?
            PoseRT rpose;
            rpose.tvec = pose.tvec.clone();
            rpose.rvec = pose.rvec.clone();
            randomize(rpose, stddev_t, stddev_r);
            cout << pose.rvec.at<double>(0, 0) << " " << rpose.rvec.at<double>(0, 0) << endl;
            cout << pose.rvec.at<double>(1, 0) << " " << rpose.rvec.at<double>(1, 0) << endl;
            cout << pose.rvec.at<double>(2, 0) << " " << rpose.rvec.at<double>(2, 0) << endl;
            
            if (visualize) { 
                PCLVisualizer visualizer;
                addPose(visualizer, pose, "pose");           
                addPose(visualizer, rpose, "rpose");
                visualizer.spin();
            }
            
            if (serialize) {
                FileStorage out(it->path().string(), FileStorage::YAML_WRITE);
                pose.write(out);
            }
        }
        it++;
    }
}

