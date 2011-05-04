/*
 * Author: Julius Adorf
 */

#include <stdlib.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <cv.h>
#include <opencv_candidate/PoseRT.h>

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace boost;
using namespace boost::algorithm;

void poseinfo(string path) {
    FileStorage in(path, FileStorage::READ);
    PoseRT pose;
    pose.read(in[PoseRT::YAML_NODE_NAME]);
    in.release();
    cout << boost::format("%10.7f %10.7f %10.7f %10.7f %10.7f %10.7f")
         % pose.tvec.at<double>(0, 0)
         % pose.tvec.at<double>(1, 0)
         % pose.tvec.at<double>(2, 0)
         % pose.rvec.at<double>(0, 0)
         % pose.rvec.at<double>(1, 0)
         % pose.rvec.at<double>(2, 0) << endl;
}

int main(int argc, char **argv) {
    // Read input arguments
    if (argc == 0) {
        cout << "Usage: poseinfo (<pose-directory>|<pose-file>)" << endl;
        return 1;
    }
    cout << boost::format("%-10s %-10s %-10s %-10s %-10s %-10s") % "t_x" % "t_y" % "t_z" % "r_x" % "r_y" % "r_z" << endl;
    for (int i = 1; i < argc; i++) {
        string path(argv[i]);
        if (boost::filesystem::is_directory(path)) {
            boost::filesystem::directory_iterator it(path);
            boost::filesystem::directory_iterator end;
            while (it != end) {
                string fname = it->filename();
                if (ends_with(fname, ".pose.yaml")) {
                    poseinfo(it->path().string());
                }
                it++;
            }
        } else {
            poseinfo(path);
        }
    }
}
