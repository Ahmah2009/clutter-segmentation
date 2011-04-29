/*
 * Author: Julius Adorf
 */

#include <tod/core/Features2d.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

using namespace std;
using namespace cv;
using namespace tod;

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << "Usage: keypoints_image <image> <features-yaml> <out-image>" << endl;
        return -1;
    }

    string img_fname(argv[1]);
    string feat_fname(argv[2]);
    string out_fname(argv[3]);

    // load image
    Mat gray = imread(img_fname, 0); 
    Mat canvas = imread(img_fname);

    // load keypoints 
    FileStorage fs(feat_fname, FileStorage::READ);
    Features2d f2d;
    f2d.read(fs[Features2d::YAML_NODE_NAME]);
    f2d.image = gray;

    // draw image and keypoints
    //f2d.draw(canvas, 0);
    drawKeypoints(f2d.image, f2d.keypoints, canvas, Scalar(0, 0, 255)); 
    imwrite(out_fname, canvas);

    return 0;
}

