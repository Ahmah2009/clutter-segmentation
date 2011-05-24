/**
 * Author: Julius Adorf
 */

#ifndef _RESULT_H_
#define _RESULT_H_

#include <map>
#include <set>
#include <string>
#include <tod/core/Features2d.h>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    struct Result {

        Result() :  guess_made(false),
                    locate_choice(),
                    detect_choices(0),
                    features() {}
        /** This constructor is designed for testing purposes */
        Result(const tod::Guess & locate_choice) :
                    guess_made(true),
                    locate_choice(locate_choice),
                    detect_choices(0),
                    features() {}
        Result(bool guess_made,
                const tod::Guess & locate_choice,
                const std::vector<tod::Guess> & detect_choices,
                const tod::Features2d & features) :
                    guess_made(guess_made),
                    locate_choice(locate_choice),
                    detect_choices(detect_choices),
                    features(features) {}
    
        /** Determines whether locate_choice is valid, or if no choice was made
         * at all. Accessing locate_choice where guess_made=false results in
         * unspecified behaviour. */
        bool guess_made;
        tod::Guess locate_choice;
        std::vector<tod::Guess> detect_choices;
        /** The extracted features from the query image are secondary results
         * and can be used for analysis. */
        tod::Features2d features; 
        std::set<std::string> distinctLabels() const;

    };

    typedef std::map<std::string, Result > SetResult;

}

#endif
