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
                            const std::string & locate_config,
                            const cv::Ptr<GuessRanking> ranking_ = new MaxInliersRanking());

            ClutSegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & locate_params,
                            const cv::Ptr<GuessRanking> ranking_ = new MaxInliersRanking());

            /** Attempts to find an object in the scene. It makes a best guess
             * according to some ranking. This algorithm proceeds in two steps.First,
             * objects are detected on the image with little regard on their exact
             * locations. High-ranked guesses are refined by applying more computing
             * resources and by using a object-specific test until the refined guess meets
             * an acceptance criterium, which is given by a ranking threshold. */
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
                        const PointCloudT & queryCloud,
                        tod::Guess & resultingGuess);

            void init(const std::string & baseDirectory,
                        cv::Ptr<GuessRanking> ranking);

            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            void loadBase();

            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters locate_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            float accept_threshold;
    };

}

#endif
