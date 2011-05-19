/**
 * Author: Julius Adorf
 */

#ifndef _RESPONSE_H_
#define _RESPONSE_H_

#include "clutseg/testdesc.h"

#include <string>
#include <tod/detecting/GuessGenerator.h>

namespace clutseg {

    /** Computes the response of the estimator on a given query.  As such the
     * result has to be compared with ground truth. The smaller the response, the
     * better the guess compared to ground truth.
     */
    struct ResponseFunction {

        virtual float operator()(const tod::Guess & guess, const GroundTruth & groundTruth);

    };

    /** Computes the response using the error measured by the sum of squares. */
    class CutSseResponseFunction : public ResponseFunction {

        public:

            CutSseResponseFunction(float max_d = 0.02, float max_a = M_PI / 9) : max_d_(max_d), max_a_(max_a) {}

            virtual float operator()(const tod::Guess & guess, const GroundTruth & groundTruth);

        private:
 
            float max_d_;
            float max_a_;

    };
}

#endif
