/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/pose.h"

#include <boost/foreach.hpp>

using namespace std;
using namespace tod;

namespace clutseg {

    float ResponseFunction::operator()(const tod::Guess & guess, const GroundTruth & groundTruth) {
        throw runtime_error("not implemented");
    }

    float CutSseResponseFunction::operator()(const Guess & guess, const GroundTruth & groundTruth) {
        PoseRT est_pose = poseToPoseRT(guess.aligned_pose());
        double r = 1.0;
        BOOST_FOREACH(NamedPose np, groundTruth) {
            if (np.name == guess.getObject()->name) {
                double dt = distBetweenLocations(est_pose, np.pose); 
                double da = angleBetweenOrientations(est_pose, np.pose); 
                double r2 = (dt * dt) / (max_d_ * max_d_) + (da * da) / (max_a_ * max_a_);
                r = r2 < r ? r2 : r;
            }
        }
        return r;
    }

}
