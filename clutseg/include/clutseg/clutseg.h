/*
 * Author: Julius Adorf
 */
#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/options.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>


namespace clutseg {

    class ClutSegmenter {
        // TODO: typedef point cloud definition away
        public:
            ClutSegmenter(const std::string & baseDirectory, const std::string & config);
            bool recognize(const cv::Mat & queryImage,
                            const pcl::PointCloud<pcl::PointXYZ> & queryCloud,
                            tod::Guess & resultingGuess,
                            pcl::PointCloud<pcl::PointXYZ> & inliersCloud);
        private:
            void loadParams();
            void loadBase();
            Options opts_; 
            tod::TrainingBase base_;
    };

}

#endif
