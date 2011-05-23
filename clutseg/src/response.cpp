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

    void ResponseFunction::operator()(const SetResult & resultSet,
                                        const SetGroundTruth & groundSet,
                                        const set<string> & templateNames,
                                        Response & rsp) {
        rsp = Response();
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
        int mislabelings = 0;
        int nones = 0;
        int tp = 0;

        sipc_t sc;
        for (SetGroundTruth::const_iterator it = groundSet.begin(); it != groundSet.end(); it++) {
            string img_name = it->first;
            GroundTruth g = it->second;
            Result r = resultSet.find(img_name)->second;

            if (g.emptyScene()) {
                sc.max_cscore += 2;
                if (r.guess_made) {
                    // False positive
                    mislabelings++;
                } else {
                    // True negative
                    sc.cscore += 2;
                }
            } else {
                sc.max_cscore++;
                sc.max_rscore++;
                sc.max_tscore++;
                if (r.guess_made) {
                    if (g.onScene(r.locate_choice.getObject()->name)) {
                        // True positive
                        vector<PoseRT> poses = g.posesOf(r.locate_choice.getObject()->name);
                        if (poses.size() > 1) {
                            throw runtime_error(
                                "ERROR: Response function does not allow for comparing \n"
                                "test result with ground truth, when there are multiple \n"
                                "instances of the same template object on the scene.");
                        }
                        PoseRT truep = poses[0];
                        Pose estp = r.locate_choice.aligned_pose();
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
                        tp++;
                    } else {
                        // False positive
                        mislabelings++;
                    }
                } else {
                    // False negative
                    nones++;
                }
            }
    
            set<string> choice_labels = r.distinctLabels();
            BOOST_FOREACH(const string & subj, templateNames) {
                if (g.onScene(subj)) {
                    if (choice_labels.count(subj) == 1) {
                        rsp.detect_tp++;
                    } else {
                        rsp.detect_fn++;
                    }
                } else {
                    if (choice_labels.count(subj) == 0) {
                        rsp.detect_tn++;
                    } else {
                        rsp.detect_fp++;
                    }
                }
            }


            sc.frames++;
        }
        
        // TODO: store them in the db
        sc.compute_final_score();
        rsp.sipc_score = sc; 

        int n = groundSet.size();
        int tps = tp;
        // 'successes' might as well be zero. In that case, we cannot compute
        // the average errors. NaN will be an appropriate value.
        rsp.avg_angle_err = acc_angle_err / tps;
        rsp.avg_succ_angle_err = acc_succ_angle_err / successes;
        rsp.avg_trans_err = acc_trans_err / tps;
        rsp.avg_succ_trans_err = acc_succ_trans_err / successes;
        rsp.avg_angle_sq_err = acc_angle_sq_err / tps;
        rsp.avg_succ_angle_sq_err = acc_succ_angle_sq_err / successes;
        rsp.avg_trans_sq_err = acc_trans_sq_err / tps;
        rsp.avg_succ_trans_sq_err = acc_succ_trans_sq_err / successes;
        rsp.succ_rate = float(successes) / n;
        rsp.mislabel_rate = float(mislabelings) / n;
        rsp.none_rate = float(nones) / n;
    }


    void CutSseResponseFunction::operator()(const SetResult & resultSet, const SetGroundTruth & groundSet, const set<string> & templateNames, Response & rsp) {
        ResponseFunction::operator()(resultSet, groundSet, templateNames, rsp);

        // TODO: maybe we can compute this directly
        double r_acc = 0;
        for (SetGroundTruth::const_iterator it = groundSet.begin(); it != groundSet.end(); it++) {
            const string & img_name = it->first;
            cout << "[RESPONSE] Validating results against ground truth: " << img_name << endl;
            const GroundTruth & g = it->second;
            const Result & result = resultSet.find(img_name)->second;
            if (!result.guess_made) {
                if (!g.emptyScene()) {
                    r_acc += 1.0;
                }
            } else {
                Pose estp = result.locate_choice.aligned_pose();
                double r = 1.0;
                vector<PoseRT> poses = g.posesOf(result.locate_choice.getObject()->name);
                BOOST_FOREACH(const PoseRT & truep, poses) {
                    double dt = dist_between(estp, truep); 
                    double da = angle_between(estp, truep); 
                    double r2 = (dt * dt) / (max_trans_error_ * max_trans_error_) + (da * da) / (max_angle_error_ * max_angle_error_);
                    r = r2 < r ? r2 : r;
                }
                r_acc += r;
            }
        }
        rsp.value = r_acc / groundSet.size();
    }

}
