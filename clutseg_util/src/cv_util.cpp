/*
 * Author: Julius Adorf
 */

#include "cv_util.h"

using namespace std;
using namespace cv;

namespace clutseg {

    void putMultilineText(Mat& outImg, const vector<string> & lines,
                        const Point & org, int fontFace, double fontScale,
                        const Scalar & color) {
        int baseline = 0;
        for (size_t i = 0; i < lines.size(); i++) {
            Size sz = getTextSize(lines[i], fontFace, fontScale, 1, &baseline);
            putText(outImg, lines[i], org + Point(0, 1.7*(i+1)*sz.height),
                    fontFace, fontScale, Scalar(255, 255, 255), 1, CV_AA, false);
        }
    }

 }

