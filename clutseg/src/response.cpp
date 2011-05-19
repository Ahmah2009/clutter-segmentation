/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/pose.h"

#include <boost/foreach.hpp>

using namespace std;
using namespace tod;

namespace clutseg {


    void ResponseFunction::operator()(const TestSetResult & result, const TestSetGroundTruth & ground, Response & response) {
        throw runtime_error("not implemented");
    }

    void CutSseResponseFunction::operator()(const TestSetResult & result, const TestSetGroundTruth & ground, Response & response) {
        ResponseFunction::operator()(result, ground, response);

        double r_acc = 0;
        for (TestSetGroundTruth::const_iterator it = ground.begin(); it != ground.end(); it++) {
            const string & img_name = it->first;
            if (result.find(img_name) == result.end()) {
                if (!ground.empty()) {
                    r_acc += 1.0;
                }
            } else {
                const Guess & guess = result.find(img_name)->second;
                PoseRT est_pose = poseToPoseRT(guess.aligned_pose());
                double r = 1.0;
                const GroundTruth & groundTruth = it->second;
                BOOST_FOREACH(NamedPose np, groundTruth) {
                    if (np.name == guess.getObject()->name) {
                        double dt = distBetweenLocations(est_pose, np.pose); 
                        double da = angleBetweenOrientations(est_pose, np.pose); 
                        double r2 = (dt * dt) / (max_trans_error_ * max_trans_error_) + (da * da) / (max_angle_error_ * max_angle_error_);
                        r = r2 < r ? r2 : r;
                    }
                }
                r_acc += r;
            }
        }
        response.value = r_acc / ground.size();
    }

}
