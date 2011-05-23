/**
 * Author: Julius Adorf
 */

#ifndef _RESULT_H_
#define _RESULT_H_

#include <map>
#include <set>
#include <string>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    struct Result {

        Result() : guess_made(false), locate_choice(), detect_choices(0) {}
        /** This constructor is designed for testing purposes */
        Result(const tod::Guess & locate_choice) :
                    guess_made(true), locate_choice(locate_choice), detect_choices(0) {}
        Result(bool guess_made, const tod::Guess & locate_choice,
                const std::vector<tod::Guess> & detect_choices) :
                    guess_made(guess_made), locate_choice(locate_choice), detect_choices(detect_choices) {}

        bool guess_made;
        tod::Guess locate_choice;
        std::vector<tod::Guess> detect_choices;

        std::set<std::string> distinctLabels() const;

    };

    typedef std::map<std::string, Result > SetResult;

}

#endif
