/**
 * Author: Julius Adorf
 */

#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/options.h"
#include "clutseg/common.h"
#include "clutseg/paramsel.h"
#include "clutseg/ranking.h"

#include <cv.h>
#include <limits>
#include <pcl/point_types.h>
#include <set>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Recognizer.h>

namespace clutseg {

    class ClutSegmenter {
        
        public:

            /** Constructed segmenter will be invalid and cannot be used. */
            ClutSegmenter();

            ClutSegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & locate_config,
                            const cv::Ptr<GuessRanking> ranking_ = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity());

            ClutSegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & locate_params,
                            const cv::Ptr<GuessRanking> ranking = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity());

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

            /** Retrieves parameters used for detection stage. Writes to the
             * parameters are transparent to the segmenter. */
            tod::TODParameters & getDetectParams();

            /** Retrieves parameters used for locating stage. Writes to the
             * parameters are transparent to the segmenter. */
            tod::TODParameters & getLocateParams();

            int getAcceptThreshold() const;

            void setAcceptThreshold(int accept_threshold);

            cv::Ptr<GuessRanking> getRanking() const;

            void setRanking(const cv::Ptr<GuessRanking> & ranking);

            /** Gets a set of template objects this segmenter knows, such as
              * assam_tea, haltbare_milch, and so on. */
            std::set<std::string> getTemplateNames() const;

            void reconfigure(const Paramset & params);

        private:

            bool detect(tod::Features2d & query,
                        std::vector<tod::Guess> & guesses);

            bool locate(const tod::Features2d & query,
                        const PointCloudT & queryCloud,
                        tod::Guess & resultingGuess);

            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            void loadBase();

            bool initialized_;
            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters locate_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            float accept_threshold_;

    };

}

#endif
