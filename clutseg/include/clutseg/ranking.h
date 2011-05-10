/**
 * Author: Julius Adorf
 */

#include <cv.h>
#include <tod/detecting/GuessGenerator.h>

// IDEA: create ranking by ratio between inliers and object matches
// IDEA: create ranking by ratio between inliers and object keypoints 

namespace clutseg {

    /** A score function that measures "goodness" of a guess. The measure is an
     * estimator and can be learned on ground truth or based on certain
     * assumptions. Rather than defining an order on the set of guesses, this
     * measure assigns real values to a guess. This allows for example for
     * combination of several measures (i.e. using a linear combination). */
    struct GuessRanking {
        
        virtual ~GuessRanking() {}

        virtual float operator()(const tod::Guess & guess) const = 0;

    };

    /** A comparator based on the ranking function, such that the guess with the
     * highest ranking will become the first container item after calling
     * std::sort. */
    class GuessComparator {

        public:

            GuessComparator();

            GuessComparator(const cv::Ptr<GuessRanking> & ranking);

            bool operator()(const tod::Guess & a, const tod::Guess & b) const;
            
        private:

            cv::Ptr<GuessRanking> ranking_;

    };

    /** Ranks guesses according to the number of inliers. The more inliers, the
     * better the score. In fact, this implementation just returns the number of
     * guesses and as such is not normalized. */
    struct MaxInliersRanking : public GuessRanking {

        float operator()(const tod::Guess & guess) const;

    };

    /** Ranks guesses according to a priori knowledge about the distribution of
     * objects in the query scene. For example, if a robot goes shopping, it
     * usually knows the items it bought. This knowledge is modeled as
     * probabilities (in the Bayesian sense) that an object is visible in the
     * scene. Guesses are ranked by the a priori probability of the recognized
     * object to be on the scene. NOT implemented yet.
     */
    struct APrioriRanking : public GuessRanking {

        float operator()(const tod::Guess & guess) const;

    };

    /** Assigns the same score for each guess. */
    struct UniformRanking : public GuessRanking {

        float operator()(const tod::Guess & guess) const;

    };

    /* Combines two ranking functions by multiplication. NOT implemented yet */
    struct ProductRanking : public GuessRanking {

        float operator()(const tod::Guess & guess) const;

    };

}

