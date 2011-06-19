/**
 * Author: Julius Adorf
 */

#ifndef _RESULT_H_
#define _RESULT_H_

#include "clutseg/common.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <map>
    #include <set>
    #include <string>
    #include <tod/core/Features2d.h>
    #include <tod/detecting/GuessGenerator.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    struct Result {

        Result() :  guess_made(false),
                    refine_choice(),
                    detect_choices(0),
                    features() {}
        /** This constructor is designed for testing purposes */
        Result(const tod::Guess & refine_choice) :
                    guess_made(true),
                    refine_choice(refine_choice),
                    detect_choices(0),
                    features() {}
        Result(bool guess_made,
                const tod::Guess & refine_choice,
                const std::vector<tod::Guess> & detect_choices,
                const tod::Features2d & features) :
                    guess_made(guess_made),
                    refine_choice(refine_choice),
                    detect_choices(detect_choices),
                    features(features) {}
    
        /** Determines whether refine_choice is valid, or if no choice was made
         * at all. Accessing refine_choice where guess_made=false results in
         * unspecified behaviour. */
        bool guess_made;
        tod::Guess refine_choice;
        std::vector<tod::Guess> detect_choices;
        /** The extracted features from the query image are secondary results
         * and can be used for analysis. They are kind of "leaked" by the recognizer
         * though they are implementation details. */
        tod::Features2d features; 

        std::set<std::string> distinctLabels() const;

    };

    struct ClutsegQuery {
        
        ClutsegQuery() : img(), cloud() {}
        ClutsegQuery(const cv::Mat & img,
                        const PointCloudT & cloud) :
                            img(img),
                            cloud(cloud) {}
        ClutsegQuery(const PointCloudT & cloud) :
                            img(img),
                            cloud(cloud) {}

        cv::Mat img;
        PointCloudT cloud;

    };

    typedef std::map<std::string, Result > SetResult;

}

#endif
