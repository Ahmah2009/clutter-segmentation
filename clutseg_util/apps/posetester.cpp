/*
 * Author: Julius Adorf
 *
 * This tool tries to detect pose in a set of images and prints out statistics,
 * especially whether pose could be identified from every single image. The
 * purpose is to measure which kind of image preprocessing might be useful to
 * make pose estimation more reliable. Pose estimation based on the
 * checkerboard requires TOD to segment out the board and detect corners. If
 * the corners cannot be detected, pose estimation will fail.
 *
 * Given a folder this program tries to find the checkerboard and reports on
 * every single image whether pose estimation failed or succeeded. As a summary
 * it prints the observed success rate.
 */

#include "pose_util.h"

#include <iostream>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_candidate/PoseRT.h>
#include <opencv_candidate/Camera.h>
#include <fiducial/fiducial.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace fiducial;
using namespace clutseg;

int main(int argc, char **argv) {
    string camera_file = "camera.yml";
    string fiducial_file = "fiducial.yml";
    if (!boost::filesystem::exists(camera_file)) {
        cerr << "No camera file '" << camera_file << "' found." << endl;
        return -1;
    }
    if (!boost::filesystem::exists(fiducial_file)) {
        cerr << "No fiducial file '" << fiducial_file << "' found." << endl;
        return -1;
    }

    Camera camera(camera_file, (Camera::CalibrationFormat) Camera::TOD_YAML);
    FileStorage fs(fiducial_file, FileStorage::READ);
    Fiducial f;
    f.read(fs["fiducial"]);
    FiducialPoseEstimator posest(f, camera, false);
    int success_cnt = 0;
    for (int i = 1; i < argc; i++) {
        Mat img = imread(argv[i]);
        PoseRT pose = posest.estimatePose(img);
        if (pose.estimated) {
            cout << "[SUCCESS] " << argv[i] << endl;
            success_cnt += 1;
            Mat imgout = imread(string(argv[i]) + ".pose.png");
            drawPose(imgout, pose, camera);
            imwrite(string(argv[i]) + ".pose.png", imgout); // TODO: extract variable
        } else {
           cout << "[FAILURE] " << argv[i] << endl;
        }
    }
    cout << endl;
    cout << boost::format("Pose estimated: %d out of %d (%5.3f %%)") %success_cnt % (argc - 1) % (100.0 * success_cnt / (double) (argc-1)) << endl;
    return 0;
}

