/**
 * Author: Julius Adorf
 */

#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/common.h"
#include "clutseg/paramsel.h"
#include "clutseg/options.h"
#include "clutseg/ranking.h"
#include "clutseg/result.h"
	
#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
    #include <limits>
    #include <pcl/point_types.h>
    #include <set>
    #include <tod/detecting/GuessGenerator.h>
    #include <tod/detecting/Recognizer.h>
    #include <vector>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /** Collection of statistics for Clutsegmenter. Serves as an accumulator,
     * and includes counts for later averaging. Be careful, even variables such as
     * detect_fp_rate are accumulators. */
    struct ClutsegmenterStats {

        ClutsegmenterStats() :
            queries(0),
            acc_keypoints(0),
            acc_detect_matches(0),
            acc_detect_guesses(0),
            acc_detect_inliers(0),
            acc_detect_choice_matches(0),
            acc_detect_choice_inliers(0),
            acc_locate_matches(0),
            acc_locate_inliers(0),
            acc_locate_guesses(0),
            acc_locate_choice_matches(0),
            acc_locate_choice_inliers(0),
            choices(0) {}

        long queries;
        long acc_keypoints;
        long acc_detect_matches;
        long acc_detect_guesses;
        long acc_detect_inliers;
        long acc_detect_choice_matches;
        long acc_detect_choice_inliers;
        long acc_locate_matches;
        long acc_locate_inliers;
        long acc_locate_guesses;
        long acc_locate_choice_matches;
        long acc_locate_choice_inliers;
        long choices;

        /** Computes average statistics, and stores them into a response
         * object. This function merges locally accumulated statistics into the global
         * response statistics. */
        void populateResponse(Response & r) const;

    };

    class Clutsegmenter {
        
        public:

            /** Constructed segmenter will be invalid and cannot be used. */
            Clutsegmenter();

            Clutsegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & locate_config,
                            const cv::Ptr<GuessRanking> ranking_ = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_refine = true);

            Clutsegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & locate_params,
                            const cv::Ptr<GuessRanking> ranking = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_refine = true);

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

            void setDoRefine(bool do_refine);

            bool isDoRefine() const;

            /** Gets a set of template objects this segmenter knows, such as
              * assam_tea, haltbare_milch, and so on. */
            std::set<std::string> getTemplateNames() const;

            void reconfigure(const Paramset & params);
        
            void resetStats();

            ClutsegmenterStats getStats() const;

            /** Attempts to find an object in the scene. It makes a best guess
             * according to some ranking. This algorithm proceeds in two steps.First,
             * objects are detected on the image with little regard on their exact
             * locations. High-ranked guesses are refined by applying more computing
             * resources and by using a object-specific test until the refined guess meets
             * an acceptance criterium, which is given by a ranking threshold. */
            void recognize(const ClutsegQuery & query, Result & result);

        private:

            bool detect(tod::Features2d & queryF2d,
                        std::vector<tod::Guess> & detectChoices,
                        std::vector<std::pair<int, int> > & matches);

            bool refine(const tod::Features2d & queryF2d,
                        const PointCloudT & queryCloud,
                        tod::Guess & locateChoice,
                        std::vector<std::pair<int, int> > & matches);

            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            void loadBase();

            ClutsegmenterStats stats_;
            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters locate_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            float accept_threshold_;
            bool do_refine_;
            bool initialized_;

    };

}

#endif
