/** 
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <cassert>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

void print_help() {
    cerr << "Usage: ground_truth_collector <test-dir> <template-min-x> <template-zero-x> <template-max-x>  [--verbose]" << endl;
}

void drawLabelAtOrigin(Mat & canvas, const PoseRT & pose, const Camera & camera, const string & label) {
    Point topleft = projectOrigin(pose, camera); 
    vector<string> legend;
    legend.push_back(label);
    drawText(canvas, legend, topleft + Point(10, 10), FONT_HERSHEY_SIMPLEX, 1, 2, Scalar::all(-1));
}

int main(int argc, char **argv) {
    if (argc != 5 && argc != 6) {
        print_help();
        return -1;
    }

    bfs::path test_dir = string(argv[1]);    
    string tpl_min(argv[2]);
    string tpl_zero(argv[3]);
    string tpl_max(argv[4]);
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
            PoseRT pose_zero;
            cout << "[GENERAL] Reading zero pose file " << it->string() << endl;
            readPose(it->string(), pose_zero);
            Mat t_min = (Mat_<double>(3, 1) << -0.15, 0, 0);
            Mat t_max = (Mat_<double>(3, 1) << 0.13, 0, 0);
            PoseRT pose_min = translatePose(pose_zero, t_min);    
            PoseRT pose_max = translatePose(pose_zero, t_max);    
            string img_name = it->filename().substr(0, it->filename().size() - 10);
            if (verbose) {
                Mat canvas3;
                canvas3 = imread((test_dir / img_name).string());
                Camera camera = Camera((test_dir / "camera.yml").string(), Camera::TOD_YAML);
                assert(bfs::exists(test_dir / "camera.yml"));
                drawLabelAtOrigin(canvas3, pose_min, camera, tpl_min); 
                drawLabelAtOrigin(canvas3, pose_zero, camera, tpl_zero); 
                drawLabelAtOrigin(canvas3, pose_max, camera, tpl_max); 
                drawPose(canvas3, pose_min, camera);
                drawPose(canvas3, pose_zero, camera);
                drawPose(canvas3, pose_max, camera);
                imshow("TestTranslatePose", canvas3);
                waitKey(0);
            }
            bfs::path ground_truth_file = test_dir / (img_name + ".ground.yaml");
            cout << "[GENERAL] Writing ground truth file " << ground_truth_file.string() << endl;
            FileStorage fs(ground_truth_file.string(), FileStorage::WRITE);
            fs << tpl_min;
            pose_min.write(fs);
            fs << tpl_zero;
            pose_zero.write(fs);
            fs << tpl_max;
            pose_max.write(fs);
            fs.release();
        }
        it++;
    }

}

