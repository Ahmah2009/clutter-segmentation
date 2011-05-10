/**
 * Author: Julius Adorf
 */

#include "clutseg/ranking.h"

using namespace cv;
using namespace std;
using namespace tod;

namespace clutseg {

    GuessComparator::GuessComparator() : ranking_(new MaxInliersRanking()) {}

    GuessComparator::GuessComparator(const Ptr<GuessRanking> & ranking) : ranking_(ranking) {}

    bool GuessComparator::operator()(const Guess & a, const Guess & b) const {
        return (*ranking_)(a) > (*ranking_)(b);
    }

    float UniformRanking::operator()(const Guess & guess) const {
        return 1.0;
    }

    float MaxInliersRanking::operator()(const Guess & guess) const {
        return guess.inliers.size();
    }

    APrioriRanking::APrioriRanking(map<string, float> apriori_density) :
                                    apriori_density_(apriori_density) {}

    float APrioriRanking::operator()(const Guess & guess) const {
        map<string, float>::const_iterator it = apriori_density_.find(guess.getObject()->name);
        if (it == apriori_density_.end()) {
            return 0.0;
        } else {
            return it->second;
        }
    }

    ProductRanking::ProductRanking(const Ptr<GuessRanking> & ranking1,
                                    const Ptr<GuessRanking> & ranking2) :
                                   ranking1_(ranking1), ranking2_(ranking2) {}

    float ProductRanking::operator()(const Guess & guess) const {
        return (*ranking1_)(guess) * (*ranking2_)(guess);
    }

}
