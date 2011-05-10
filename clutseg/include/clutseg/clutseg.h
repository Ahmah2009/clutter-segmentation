/*
 * Author: Julius Adorf
 */
#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/options.h"
#include "clutseg/common.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>

namespace clutseg {

    class ClutSegmenter {
        
        public:

            ClutSegmenter(const std::string & baseDirectory,
                            const std::string & config);

            ClutSegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & refine_config);

            bool recognize(const cv::Mat & queryImage,
                            const PointCloudT & queryCloud,
                            tod::Guess & resultingGuess,
                            PointCloudT & inliersCloud);

        private:

            bool refine(const tod::Features2d & query,
                            const PointCloudT & queryCloud,
                            tod::Guess & resultingGuess,
                            PointCloudT & inliersCloud);

            void loadParams(Options & opts);

            void loadBase();

            Options detect_opts_; 
            Options refine_opts_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
    };

}

#endif
