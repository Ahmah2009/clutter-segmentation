/*
 * Author: Julius Adorf
 */

#ifndef _CLUTSEG_H_
#define _CLUTSEG_H_

#include "clutseg/common.h"
#include "clutseg/paramsel.h"
#include "clutseg/options.h"
#include "clutseg/query.h"
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

    /** \brief Accumulator object of statistics for clutseg:Clutsegmenter. */
    struct ClutsegmenterStats {

        ClutsegmenterStats() :
            queries(0),
            acc_keypoints(0),
            acc_detect_matches(0),
            acc_detect_guesses(0),
            acc_detect_inliers(0),
            acc_detect_choice_matches(0),
            acc_detect_choice_inliers(0),
            acc_refine_matches(0),
            acc_refine_inliers(0),
            acc_refine_guesses(0),
            acc_refine_choice_matches(0),
            acc_refine_choice_inliers(0),
            choices(0) {}

        long queries;
        long acc_keypoints;
        long acc_detect_matches;
        long acc_detect_guesses;
        long acc_detect_inliers;
        long acc_detect_choice_matches;
        long acc_detect_choice_inliers;
        long acc_refine_matches;
        long acc_refine_inliers;
        long acc_refine_guesses;
        long acc_refine_choice_matches;
        long acc_refine_choice_inliers;
        long choices;

        /**
         * \brief Computes average statistics, and stores them into a response
         * object.
         */
        void populateResponse(Response & r) const;

    };

    /** \brief Recognizes an object in a query scene. */
    class Clutsegmenter {
        
        public:

            /** \brief Dummy constructor; constructed segmenter will be invalid
             * and may not be used. */
            Clutsegmenter();

            // TODO: rename baseDirectory => modelbase_dir
   
            /**
             * \brief Constructs a Clutsegmenter for recognizing objects in a
             * query scene.
             *
             * Reads models from baseDirectory and reads detect and refine
             * configurations from [baseDirectory]/detect.config.yaml and
             * [baseDirectory]/refine.config.yaml and uses default values for
             * accept_threshold and the ranking. If tar == true, then
             * baseDirectory is interpreted as a tar file. */ 
            Clutsegmenter(const std::string & baseDirectory, bool tar = false);

            /** \brief Constructs a Clutsegmenter for recognizing objects in a
             * query scene. */
            Clutsegmenter(const std::string & baseDirectory,
                            const std::string & detect_config,
                            const std::string & refine_config,
                            const cv::Ptr<GuessRanking> ranking_ = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_refine = true);

            /** \brief Constructs a Clutsegmenter for recognizing objects in a
             * query scene. */
            Clutsegmenter(const std::string & baseDirectory,
                            const tod::TODParameters & detect_params,
                            const tod::TODParameters & refine_params,
                            const cv::Ptr<GuessRanking> ranking = new InliersRanking(),
                            float accept_threshold = -std::numeric_limits<float>::infinity(),
                            bool do_refine = true);

            /** \brief Returns a reference to the parameters used in the
             * detection stage.  Changes to these parameters are transparent to
             * the segmenter. */
            tod::TODParameters & getDetectParams();

            /** \brief Returns a reference to the parameters used in the
             * refinement stage.  Changes to these parameters are transparent
             * to the segmenter. */
            tod::TODParameters & getRefineParams();

            int getAcceptThreshold() const;

            void setAcceptThreshold(int accept_threshold);

            /** \brief Returns the guess ranking used for both the detection stage and the refinement stage. */
            cv::Ptr<GuessRanking> getRanking() const;

            /** \brief See Clutsegmenter::getRanking. */
            void setRanking(const cv::Ptr<GuessRanking> & ranking);

            /** \brief See Clutsegmenter::isDoRefine. */
            void setDoRefine(bool do_refine);

            /** \brief If true, the refinement stage is enabled.
             *
             * If false, the refinement stage is disabled; the initial guess in
             * the detection stage is returned directly.
             */
            bool isDoRefine() const;

            /** \brief Returns a set of template objects this segmenter knows,
             * such as assam_tea, haltbare_milch, icedtea, ... . */
            std::set<std::string> getTemplateNames() const;

            /**
             * \brief Configures the parameter values.
             *
             * Convenient if the parameter set is read from a database.
             */
            void reconfigure(const Paramset & params);
       
            /**
             * \brief Resets the accumulator object used for collecting statistics.
             *
             * See Clutsegmenter::ClutsegmenterStats. After performing an experiment on a
             * batch of images, for which average statistics are obtained, the
             * accumulator object must be reset.
             */ 
            void resetStats();
            
            /**
             * \brief Gets the accumulator object that contains the statistics, 
             * recorded since the last call to Clutsegmenter::resetStats.
             */
            ClutsegmenterStats getStats() const;

            /**
             * \brief Finds an object in the scene.
             *
             * Requires a query image and returns the estimated pose for an
             * object.  The detection stage yields a set of initial guesses.
             * The highest-ranked initial guess is improved in the refinement
             * stage.
             */
            void recognize(const Query & query, Result & result);

        private:

            /** \brief Detection stage. */
            bool detect(tod::Features2d & queryF2d,
                        std::vector<tod::Guess> & detectChoices,
                        std::vector<std::pair<int, int> > & matches);

            /** \brief Refinement stage. */
            bool refine(const tod::Features2d & queryF2d,
                        const PointCloudT & queryCloud,
                        tod::Guess & refineChoice,
                        std::vector<std::pair<int, int> > & matches);

            /** \brief Load parameters from file. */
            void loadParams(const std::string & config,
                            tod::TODParameters & params);

            /** \brief Load modelbase. */
            void loadBase();

            ClutsegmenterStats stats_;
            std::string baseDirectory_;
            tod::TODParameters detect_params_; 
            tod::TODParameters refine_params_; 
            tod::TrainingBase base_;
            std::vector<cv::Ptr<tod::TexturedObject> > objects_;
            cv::Ptr<GuessRanking> ranking_;
            float accept_threshold_;
            bool do_refine_;
            bool initialized_;

    };

}

#endif
