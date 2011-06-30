/*
 * Author: Julius Adorf
 */

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <cv.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <tod/core/Features2d.h>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace std;
using namespace cv;
using namespace tod;
namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << "Usage: keypoints_image <image> <features-yaml> <out-image>" << endl;
        return -1;
    }

    bfs::path img_fname(argv[1]);
    bfs::path feat_fname(argv[2]);
    bfs::path out_fname(argv[3]);

    // load image
    Mat gray = imread(img_fname.string(), 0); 
    Mat canvas = imread(img_fname.string());

    // load keypoints 
    FileStorage fs(feat_fname.string(), FileStorage::READ);
    Features2d f2d;
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = gray;

    // draw image and keypoints
    //f2d.draw(canvas, 0);
    drawKeypoints(f2d.image, f2d.keypoints, canvas, Scalar(0, 0, 255)); 
    imwrite(out_fname.string(), canvas);

    return 0;
}

