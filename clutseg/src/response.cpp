/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/pose.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <limits>

using namespace opencv_candidate;
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
        // In case, the angular and translational error both are less than a given threshold.
        int successes = 0;
        // Mislabelings are false positives in the bag of words classification
        // model. For example, if icedtea does not show up on the scene but is
        // said to be on the scene by the recognizer, then we call this a
        // "mislabeling".
        int mislabelings = 0;
        // Nones count the number of scenes where no choice was made,
        // independently whether there was any object on the scene or not
        int nones = 0;
        // This is the number of cases in which calculating pose error makes sense.
        int v = 0;
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
                        v++;
                        PoseRT truep = poses[0];
                        PoseRT estp = poseToPoseRT(c.aligned_pose());
                        double t = dist_between(estp, truep); 
                        double a = angle_between(estp, truep); 
                        acc_angle_err += abs(a);
                        acc_angle_sq_err += a * a;
                        acc_trans_err += abs(t);
                        acc_trans_sq_err += t * t;
                        // FIXME: these are directly parameter-dependent measures,
                        //        put max_angle_error and max_trans_error somewhere
                        if (a <= M_PI / 9 && t <= 0.03) {
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
                nones++;
                if (g.emptyScene()) {
                    // no choice made and scene empty
                } else { 
                    // no choice made, yet scene is showing objects 
                    // doing nothing means decreasing success_rate
                }
            }

        }
        
        int n = ground.size();
        // FIXME: Nones are problems, they pull down average though bad! Need to 
        //        ignore them in averaging!
        // v may well be zero. In that case, the fields will be assigned NAN,
        // which is best way to handle it (we have no data to calculate the error, so 
        // NAN is appropriate).
        resp.avg_angle_err = acc_angle_err / v;
        resp.avg_succ_angle_err = acc_succ_angle_err / successes;
        resp.avg_trans_err = acc_trans_err / v;
        resp.avg_succ_trans_err = acc_succ_trans_err / successes;
        resp.avg_angle_sq_err = acc_angle_sq_err / v;
        resp.avg_succ_angle_sq_err = acc_succ_angle_sq_err / successes;
        resp.avg_trans_sq_err = acc_trans_sq_err / v;
        resp.avg_succ_trans_sq_err = acc_succ_trans_sq_err / successes;
        resp.succ_rate = float(successes) / n;
        resp.mislabel_rate = float(mislabelings) / n;
        resp.none_rate = float(nones) / n;
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
                        double dt = dist_between(est_pose, np.pose); 
                        double da = angle_between(est_pose, np.pose); 
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
