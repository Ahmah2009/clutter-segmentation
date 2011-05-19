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

    class TestSetResult {

        public:

            std::map<std::string, tod::Guess>::iterator begin() { return guesses.begin(); }
            std::map<std::string, tod::Guess>::const_iterator begin() const { return guesses.begin(); }
            std::map<std::string, tod::Guess>::iterator end() { return guesses.end(); }
            std::map<std::string, tod::Guess>::const_iterator end() const { return guesses.end(); }

            bool guessMade(const std::string & img_name) const { return guesses.find(img_name) != end(); }
            const tod::Guess & get(const std::string & img_name) const { return guesses.find(img_name)->second; }
            void put(const std::string & img_name, const tod::Guess & guess) { guesses[img_name] = guess; }

        private:

            std::map<std::string, tod::Guess> guesses;

    };

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
            virtual void operator()(const TestSetResult & result,
                                    const TestSetGroundTruth & ground,
                                    Response & response);

            float max_trans_error_;
            float max_angle_error_;

    };

    /** Computes the response using the error measured by the sum of squares. */
    class CutSseResponseFunction : public ResponseFunction {

        public:

            CutSseResponseFunction(float max_trans_error = 0.02, float max_angle_error = M_PI / 9) : ResponseFunction(max_trans_error, max_angle_error) {}

            virtual void operator()(const TestSetResult & result,
                                    const TestSetGroundTruth & ground,
                                    Response & response);

    };
}

#endif
