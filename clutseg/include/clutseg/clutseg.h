/*
 * Author: Julius Adorf
 */
#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/options.h"
#include "clutseg/common.h"
#include "clutseg/ranking.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>

namespace clutseg {

    class ClutSegmenter {
        
        public:

            #ifdef TEST
                /** The no-argument constructor is required by Google test
                  * fixtures. It shall not be used in production-code. */ 
                ClutSegmenter();
            #endif

            ClutSegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & locate_config);

            ClutSegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & locate_params);

            bool recognize(const cv::Mat & queryImage,
                            const PointCloudT & queryCloud,
                            tod::Guess & resultingGuess,
                            PointCloudT & inliersCloud);

            tod::TODParameters & getDetectParams();

            tod::TODParameters & getLocateParams();

        private:

            bool detect(tod::Features2d & query,
                        std::vector<tod::Guess> & guesses);

            bool locate(const tod::Features2d & query,
                            tod::Guess & resultingGuess);

            void init(const std::string & baseDirectory);

            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            void loadBase();

            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters locate_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            int accept_threshold;
    };

}

#endif
