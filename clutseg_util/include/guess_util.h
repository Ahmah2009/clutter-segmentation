/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <tod/core/TrainingBase.h>
#include <tod/core/Features2d.h>
#include <tod/detecting/Matcher.h>

using namespace cv;
using namespace std;
using namespace tod;

namespace clutseg {

    // TODO: populate canvas instead of returning Matrix
    Mat drawAllMatches(Mat canvas, const TrainingBase & base,
                            const Ptr<Matcher> matcher, const Mat& testImage,
                            const KeypointVector & testKeypoints, const string & baseDirectory);

    void drawInliers(Mat & outImg, const Guess & guess, const Mat & testImage);
}

