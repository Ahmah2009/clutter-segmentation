/* 
 * Author: Julius Adorf
 */

#include "test.h"

// TODO: migrate to clutseg_util

#include "clutseg/viz.h"
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

using namespace cv;
using namespace clutseg;

TEST(CvDraw, TestCoordinates) {
    Mat canvas(200, 200, CV_8UC3);
    Point x1(10, 10);
    Point x2(100, 10);
    Point y1(10, 10);
    Point y2(10, 100);
    line(canvas, x1, x2, Scalar(0, 0, 255));
    line(canvas, y1, y2, Scalar(255, 0, 0));
    putText(canvas, "10, 10", Point(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1, CV_AA, false);
    putText(canvas, "100, 10", Point(100, 10), CV_FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1, CV_AA, false);
    putText(canvas, "10, 100", Point(10, 100), CV_FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1, CV_AA, false);
    putText(canvas, "10, -100", Point(10, -100), CV_FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1, CV_AA, false);
    imshow("TestCoordinates", canvas);
    waitKey(0);
}

