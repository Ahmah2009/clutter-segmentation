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

    typedef std::map<std::string, tod::Guess> TestSetResult;    

    /** Computes the response of the estimator on a given test set.  As such
     * the result has to be compared with ground truth. The smaller the response,
     * the better the guess compared to ground truth. The response function is
     * deliberately defined over the whole test set rather than for a single query
     * to allow to incorporate (and collect) global performance statistics. */
    struct ResponseFunction {

        virtual void operator()(const TestSetResult & result, const TestSetGroundTruth & ground, Response & response);

    };

    /** Computes the response using the error measured by the sum of squares. */
    class CutSseResponseFunction : public ResponseFunction {

        public:

            CutSseResponseFunction(float max_trans_error = 0.02, float max_angle_error = M_PI / 9) : max_trans_error_(max_trans_error), max_angle_error_(max_angle_error) {}

            virtual void operator()(const TestSetResult & result, const TestSetGroundTruth & ground, Response & response);

        private:
 
            float max_trans_error_;
            float max_angle_error_;

    };
}

#endif
