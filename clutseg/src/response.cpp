/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/pose.h"

#include <boost/foreach.hpp>
#include <iostream>

using namespace std;
using namespace tod;

namespace clutseg {


    void ResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & response) {
        response.value = 0.0;
    }

    void CutSseResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & response) {
        ResponseFunction::operator()(result, ground, response);

        double r_acc = 0;
        for (SetGroundTruth::const_iterator it = ground.begin(); it != ground.end(); it++) {
            const string & img_name = it->first;
            cout << "[RESPONSE] Validating results against ground truth: " << img_name << endl;
            const GroundTruth & groundTruth = it->second;
            if (!result.guessMade(img_name)) {
                if (!groundTruth.emptyScene()) {
                    r_acc += 1.0;
                }
            } else {
                Guess guess = result.get(img_name);
                PoseRT est_pose = poseToPoseRT(guess.aligned_pose());
                double r = 1.0;
                BOOST_FOREACH(const NamedPose & np, groundTruth.labels) {
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
