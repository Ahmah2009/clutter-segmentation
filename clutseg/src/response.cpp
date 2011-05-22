/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/sipc.h"
#include "clutseg/pose.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <limits>
#include <vector>

using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace clutseg {

    void ResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & rsp) {
        rsp.value = 0.0;

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
        sipc_t sc;
        for (SetGroundTruth::const_iterator it = ground.begin(); it != ground.end(); it++) {
            string img_name = it->first;
            GroundTruth g = it->second;

            if (g.emptyScene()) {
                sc.max_cscore += 2;
                if (result.guessMade(img_name)) {
                    // False positive
                    // mislabelings++;
                    sc.fp++;
                } else {
                    nones++;
                    // True negative
                    sc.tn++;
                    sc.cscore += 2;
                }
            } else {
                sc.max_cscore++;
                sc.max_rscore++;
                sc.max_tscore++;
                if (result.guessMade(img_name)) {
                    Guess c = result.get(img_name);
                    if (g.onScene(c.getObject()->name)) {
                        // True positive
                        vector<PoseRT> poses = g.posesOf(c.getObject()->name);
                        if (poses.size() > 1) {
                            throw runtime_error(
                                "ERROR: Response function does not allow for comparing \n"
                                "test result with ground truth, when there are multiple \n"
                                "instances of the same template object on the scene.");
                        }
                        PoseRT truep = poses[0];
                        // TODO: no need to convert here
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
                        
                        sc.rscore += compute_rscore(a);  
                        sc.tscore += compute_tscore(t);
                        sc.cscore++;
                        sc.tp++;
                    } else {
                        // False positive
                        // mislabelings++;
                        sc.fp++;
                    }
                } else {
                    // False negative
                    nones++;
                    sc.fn++;
                }
            }
    
            sc.frames++;
        }
        
        // TODO: store them in the db / response struct
        // rsp.sipc_score = compute_sipc_score(fscores);
        // sc.frames = fscores.size();
        // sc.final_score = 0.5 * sc.cscore + 0.25 * sc.rscore + 0.25 * sc.tscore;
        // sc.final_score /= sc.frames;
        sc.compute_final_score();
        rsp.sipc_score = sc; 

        int n = ground.size();
        // FIXME: Nones are problems, they pull down average though bad! Need to 
        //        ignore them in averaging!
        // v may well be zero. In that case, the fields will be assigned NAN,
        // which is best way to handle it (we have no data to calculate the error, so 
        // NAN is appropriate).
        int tps = rsp.sipc_score.tp;
        rsp.avg_angle_err = acc_angle_err / tps;
        rsp.avg_succ_angle_err = acc_succ_angle_err / successes;
        rsp.avg_trans_err = acc_trans_err / tps;
        rsp.avg_succ_trans_err = acc_succ_trans_err / successes;
        rsp.avg_angle_sq_err = acc_angle_sq_err / tps;
        rsp.avg_succ_angle_sq_err = acc_succ_angle_sq_err / successes;
        rsp.avg_trans_sq_err = acc_trans_sq_err / tps;
        rsp.avg_succ_trans_sq_err = acc_succ_trans_sq_err / successes;
        rsp.succ_rate = float(successes) / n;
        rsp.mislabel_rate = float(sc.fp) / n; // TODO: rename into fp_rate
        rsp.none_rate = float(sc.tn + sc.fn) / n;
    }


    void CutSseResponseFunction::operator()(const SetResult & result, const SetGroundTruth & ground, Response & rsp) {
        ResponseFunction::operator()(result, ground, rsp);

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
        rsp.value = r_acc / ground.size();
    }

}
