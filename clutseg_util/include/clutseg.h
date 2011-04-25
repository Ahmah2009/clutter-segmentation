/*
 * Author: Julius Adorf
 */
#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "options.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>

using namespace cv;
using namespace pcl;

namespace clutseg {

    typedef pcl::PointCloud<PointXYZ> PointCloudT;

    class ClutSegmenter {
        public:
            ClutSegmenter(const string & baseDirectory, const string & config);
            bool recognize(const Mat & queryImage, const PointCloudT & queryCloud, tod::Guess & resultingGuess, PointCloudT & inliersCloud);
        private:
            void loadParams();
            void loadBase();
            Options opts_; 
            tod::TrainingBase base_;
    };

}

#endif
