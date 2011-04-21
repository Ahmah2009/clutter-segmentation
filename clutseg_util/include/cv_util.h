/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <vector>

using namespace std;
using namespace cv;

namespace clutseg {

    void putMultilineText(Mat& outImg, const vector<string> & lines,
                        const Point & org, int fontFace, double fontScale,
                        const Scalar & color);

}

