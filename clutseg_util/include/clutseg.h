/*
 * Author: Julius Adorf
 */

#include "options.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>

using namespace cv;
using namespace pcl;
using namespace tod;

namespace clutseg {

    typedef pcl::PointCloud<PointXYZ> PointCloudT;

    class ClutSegmenter {

        public:

            ClutSegmenter();
            ClutSegmenter(Options & _opts, TrainingBase & _base); // : opts(_opts), base(_base);

            bool recognize(const Mat & queryImage, const PointCloudT & queryCloud, Guess & resultingGuess, PointCloudT & inliersCloud);
           
            Options opts; 
            TrainingBase base;
    };

}

