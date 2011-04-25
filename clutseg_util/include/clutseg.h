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
            ClutSegmenter(const string & baseDirectory, const string & config);
            bool recognize(const Mat & queryImage, const PointCloudT & queryCloud, Guess & resultingGuess, PointCloudT & inliersCloud);
        private:
            void loadParams();
            void loadBase();
            Options opts_; 
            TrainingBase base_;
    };

}

