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

    /** Collection of statistics for ClutSegmenter. Serves as an accumulator,
     * and includes count for later averaging. */
    struct ClutSegmenterStats {

        ClutSegmenterStats() :
            queries(0),
            keypoints(0),
            detect_matches(0),
            detect_guesses(0),
            detect_inliers(0),
            detect_choice_matches(0),
            detect_choice_inliers(0),
            locate_matches(0),
            locate_inliers(0),
            locate_guesses(0),
            locate_choice_matches(0),
            locate_choice_inliers(0),
            choices(0) {}

        long queries;
        // TODO: include runtime in response
        // clock_t runtime;
        long keypoints;
        long detect_matches;
        long detect_guesses;
        long detect_inliers;
        long detect_choice_matches;
        long detect_choice_inliers;
        long locate_matches;
        long locate_inliers;
        long locate_guesses;
        long locate_choice_matches;
        long locate_choice_inliers;
        long choices;

    };

    class ClutSegmenter {
        
        public:

            /** Constructed segmenter will be invalid and cannot be used. */
            ClutSegmenter();

            ClutSegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & locate_config,
                            const cv::Ptr<GuessRanking> ranking_ = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_locate = true);

            ClutSegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & locate_params,
                            const cv::Ptr<GuessRanking> ranking = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_locate = true);

            /** Attempts to find an object in the scene. It makes a best guess
             * according to some ranking. This algorithm proceeds in two steps.First,
             * objects are detected on the image with little regard on their exact
             * locations. High-ranked guesses are refined by applying more computing
             * resources and by using a object-specific test until the refined guess meets
             * an acceptance criterium, which is given by a ranking threshold. */
            bool recognize(const cv::Mat & queryImage,
                            const PointCloudT & queryCloud,
                            tod::Guess & choice,
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

            void setDoLocate(bool do_locate);

            bool isDoLocate() const;

            /** Gets a set of template objects this segmenter knows, such as
              * assam_tea, haltbare_milch, and so on. */
            std::set<std::string> getTemplateNames() const;

            void reconfigure(const Paramset & params);
        
            void resetStats();

            ClutSegmenterStats getStats() const;

        private:

            bool detect(tod::Features2d & query,
                        std::vector<tod::Guess> & guesses,
                        cv::Ptr<tod::Matcher> & detectMatcher);

            bool locate(const tod::Features2d & query,
                        const PointCloudT & queryCloud,
                        tod::Guess & choice,
                        cv::Ptr<tod::Matcher> & locateMatcher);

            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            void loadBase();

            ClutSegmenterStats stats_;
            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters locate_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            float accept_threshold_;
            bool do_locate_;
            bool initialized_;

    };

}

#endif
