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


    void ResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & resp) {
        resp.value = 0.0;

        float acc_angle_err = 0;
        float acc_succ_angle_err = 0;
        float acc_trans_err = 0;
        float acc_succ_trans_err = 0;
        float acc_angle_sq_err = 0;
        float acc_succ_angle_sq_err = 0;
        float acc_trans_sq_err = 0;
        float acc_succ_trans_sq_err = 0;
        int successes = 0;
        int mislabelings = 0;
        int nones = 0;
        for (SetGroundTruth::const_iterator it = ground.begin(); it != ground.end(); it++) {
            string img_name = it->first;
            GroundTruth g = it->second;
            if (result.guessMade(img_name)) {
                Guess c = result.get(img_name);
                if (g.onScene(c.getObject()->name)) {
                    // choice made for object on scene
                    vector<PoseRT> poses = g.posesOf(c.getObject()->name);
                    if (poses.size() > 1) {
                        throw runtime_error(
                            "ERROR: Response function does not allow for comparing \n"
                            "test result with ground truth, when there are multiple \n"
                            "instances of the same template object on the scene.");
                    } else {
                        PoseRT truep = poses[0];
                        PoseRT estp = poseToPoseRT(c.aligned_pose());
                        double t = distBetweenLocations(estp, truep); 
                        double a = angleBetweenOrientations(estp, truep); 
                        acc_angle_err += abs(a);
                        acc_angle_sq_err += a * a;
                        acc_trans_err += abs(t);
                        acc_trans_sq_err += t * t;
                        // FIXME: these are directly parameter-dependent measures,
                        //        put max_angle_error and max_trans_error somewhere
                        if (a <= M_PI / 9 && t <= 0.02) {
                            successes++;
                            acc_succ_angle_err += abs(a);
                            acc_succ_angle_sq_err += a * a;
                            acc_succ_trans_err += abs(t);
                            acc_succ_trans_sq_err += t * t;
                        }
                    }
                } else {
                    // choice made but for object not on scene
                    mislabelings++;
                }
            } else {
                if (g.emptyScene()) {
                    // no choice made and scene empty
                    successes++;
                } else {
                    // no choice made, yet scene is showing objects 
                    nones++;
                }
            }

        }
        
        int n = ground.size(); 
        resp.avg_angle_err = acc_angle_err / n;
        resp.avg_succ_angle_err = acc_succ_angle_err / n;
        resp.avg_trans_err = acc_trans_err / n;
        resp.avg_succ_trans_err = acc_succ_trans_err / n;
        resp.avg_angle_sq_err = acc_angle_sq_err / n;
        resp.avg_succ_angle_sq_err = acc_succ_angle_sq_err / n;
        resp.avg_trans_sq_err = acc_trans_sq_err / n;
        resp.avg_succ_trans_sq_err = acc_succ_trans_sq_err / n;
        resp.succ_rate = successes / n;
        resp.mislabel_rate = mislabelings / n;
        resp.none_rate = nones / n;
    }

    void CutSseResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & resp) {
        ResponseFunction::operator()(result, ground, resp);

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
                BOOST_FOREACH(const LabeledPose & np, groundTruth.labels) {
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
        resp.value = r_acc / ground.size();
    }

}
