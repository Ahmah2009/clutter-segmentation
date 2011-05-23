/**
 * Author: Julius Adorf
 */

#ifndef _RESPONSE_H_
#define _RESPONSE_H_

#include "clutseg/ground.h"
#include "clutseg/paramsel.h"

#include <map>
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

    };

    typedef std::map<std::string, Result > SetResult;

    /** Computes the response of the estimator on a given test set.  As such
     * the result has to be compared with ground truth. The smaller the response,
     * the better the guess compared to ground truth. The response function is
     * deliberately defined over the whole test set rather than for a single query
     * to allow to incorporate (and collect) global performance statistics. */
    class ResponseFunction {

        protected:

            ResponseFunction(float max_trans_error = 0.02, float max_angle_error = M_PI / 9) : max_trans_error_(max_trans_error), max_angle_error_(max_angle_error) {}

            /** Basic response function always sets response.value to zero, and
             * populates the other response statistics, such as response.success_rate etc.
             * This method must be called from derived classes. */
            virtual void operator()(const SetResult & result,
                                    const SetGroundTruth & ground,
                                    Response & response);

            float max_trans_error_;
            float max_angle_error_;

    };

    /** Computes the response using the error measured by the sum of squares. */
    class CutSseResponseFunction : public ResponseFunction {

        public:

            CutSseResponseFunction(float max_trans_error = 0.03, float max_angle_error = M_PI / 9) : ResponseFunction(max_trans_error, max_angle_error) {}

            virtual void operator()(const SetResult & result,
                                    const SetGroundTruth & ground,
                                    Response & response);

    };
}

#endif
