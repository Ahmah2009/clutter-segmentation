/**
 * Author: Julius Adorf
 */

#include "clutseg/modelbase.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <boost/foreach.hpp>
    #include <boost/algorithm/string.hpp>
    #include <cassert>
    #include <cv.h>
    #include <opencv2/highgui/highgui.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

void print_help() {
    cerr << "Usage: collect_ground_truth <test-dir> <object-min-x> <object-zero-x> <object-max-x>  [--verbose]" << endl;
    cerr << endl
        << "collect_ground_truth iterates over a directory and looks for pose files and" << endl
        << "corresponding images that show three objects. It calculates their respective poses." << endl
        << endl
        << "For each image, the pose estimated from fiducial markers is read. If it is not" << endl
        << "available, the image will be skipped. From this pose, the two other poses will" << endl
        << "be calculated by translating with respect to model coordinates as defined by" << endl
        << "the fiducial markers. The three poses will be associated with the specified" << endl
        << "names of the template objects, such that: " << endl
        << "    object-min-x labels template at (-0.15, 0, 0) in model coordinates " << endl
        << "    object-zero-x labels template at (0, 0, 0) in model coordinates " << endl
        << "    object-max-x labels template at (0.13, 0, 0) in model coordinates " << endl
        << "For each image, the results will be written to a <image-name>.ground.yaml file. " << endl;

}

int main(int argc, char **argv) {
    if (argc != 5 && argc != 6) {
        print_help();
        return -1;
    }

    bfs::path test_dir = string(argv[1]);
    string obj_min(argv[2]);
    string obj_zero(argv[3]);
    string obj_max(argv[4]);
    bool verbose = false;
    if (argc == 6) {
        if (string(argv[5]) == "--verbose") {
            verbose = true;
        } else {
            print_help();
            return -1;
        }
    }
    bfs::directory_iterator it(test_dir);
    bfs::directory_iterator end;
    while (it != end) {
        if (boost::ends_with(it->filename(), ".pose.yaml")) {
            LabelSet g;
            PoseRT pose_zero;
            cout << "[GENERAL] Reading zero pose file " << it->string() << endl;
            readPose(it->string(), pose_zero);
            Mat t_min = (Mat_<double>(3, 1) << -0.15, 0, 0);
            Mat t_max = (Mat_<double>(3, 1) << 0.13, 0, 0);
            PoseRT pose_min = translatePose(pose_zero, t_min);
            PoseRT pose_max = translatePose(pose_zero, t_max);
            string img_name = it->filename().substr(0, it->filename().size() - 10);

            g.labels.push_back(Label(obj_min, pose_min));
            g.labels.push_back(Label(obj_zero, pose_zero));
            g.labels.push_back(Label(obj_max, pose_max));

            bfs::path ground_truth_file = test_dir / (img_name + ".ground.yaml");
            cout << "[GENERAL] Writing ground truth file " << ground_truth_file.string() << endl;
            FileStorage fs(ground_truth_file.string(), FileStorage::WRITE);
            fs << "labels";
            g.write(fs);
            fs.release();

           if (verbose) {
                Mat canvas3;
                canvas3 = imread((test_dir / img_name).string());
                Camera camera = Camera((test_dir / "camera.yml").string(), Camera::TOD_YAML);
                assert(bfs::exists(test_dir / "camera.yml"));
                drawLabelAtOrigin(canvas3, pose_min, camera, obj_min, Scalar(0, 0, 255));
                drawLabelAtOrigin(canvas3, pose_zero, camera, obj_zero, Scalar(0, 255, 0));
                drawLabelAtOrigin(canvas3, pose_max, camera, obj_max, Scalar(255, 0, 0));
                drawPose(canvas3, pose_min, camera);
                drawPose(canvas3, pose_zero, camera);
                drawPose(canvas3, pose_max, camera);
                imshow(img_name, canvas3);
                waitKey(0);
            }
        }
        it++;
    }

}

