/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"

#include "clutseg/sipc.h"
#include "clutseg/pose.h"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <limits>
#include <vector>

using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace clutseg {


    /** Computes score for one single test scene */
    void update_detect_sipc(const Result & result,
                            const GroundTruth & ground,
                            const set<string> & templateNames,
                            detect_sipc_t & detect_sipc) {
        int s_h = 0;
        int s_m = 0;
        int s_n = 0;
        float s_r = 0;
        float s_t = 0;
        set<string> choice_labels = result.distinctLabels();
        BOOST_FOREACH(const string & subj, templateNames) {
            if (ground.onScene(subj)) {
                detect_sipc.objects++;
                if (choice_labels.count(subj) == 1) {
                    s_h++;
                    
                    vector<PoseRT> poses = ground.posesOf(subj);
                    if (poses.size() > 1) {
                        throw runtime_error(
                            "ERROR: Response function does not allow for comparing \n"
                            "test result with ground truth, when there are multiple \n"
                            "instances of the same template object on the scene.");
                    }
                    PoseRT truep = poses[0];
                    Pose estp;
                    // choose the first (best-ranked), ignore all others
                    BOOST_FOREACH(const Guess & c, result.detect_choices) {
                        if (c.getObject()->name == subj) {
                            estp = c.aligned_pose();
                            break;
                        }
                    }
                    double t = dist_between(estp, truep); 
                    double a = angle_between(estp, truep); 
                    s_t += compute_tscore(t);
                    s_r += compute_rscore(a);
                } else {
                    s_m++;
                }
            } else {
                if (choice_labels.count(subj) == 0) {
                    // just do nothing
                } else {
                    s_n++;
                }
            }
        }
        detect_sipc.acc_score += 0.5 * max(0.0, s_h - 0.5 * s_m - s_n + 0.5 *(s_r + s_t));
    }

    void compute_errors(const Guess & guess, const GroundTruth & ground, double & angle_err, double & trans_err) {
        vector<PoseRT> poses = ground.posesOf(guess.getObject()->name);
        if (poses.size() > 1) {
            throw runtime_error(
            "ERROR: Response function does not allow for comparing \n"
            "test result with ground truth, when there are multiple \n"
            "instances of the same template object on the scene.");
        }
        PoseRT truep = poses[0];
        Pose estp = guess.aligned_pose();
        angle_err = angle_between(estp, truep); 
        trans_err = dist_between(estp, truep); 
    }

    /** Computes score for one single test scene */
    void update_locate_sipc(const Result & result,
                            const GroundTruth & ground,
                            const set<string> & /* templateNames */,
                            locate_sipc_t & locate_sipc) {
        if (ground.emptyScene()) {
            if (result.guess_made) {
                // False positive
                // Do nothing 
            } else {
                // True negative
                locate_sipc.cscore += 2;
            }
        } else {
            if (result.guess_made) {
                Guess lc = result.locate_choice;
                if (ground.onScene(lc.getObject()->name)) {
                    // True positive
                    double a;
                    double t;
                    compute_errors(lc, ground, a, t);
                    locate_sipc.rscore += compute_rscore(a);  
                    locate_sipc.tscore += compute_tscore(t);
                    locate_sipc.cscore++;
                } else {
                    // False positive
                    // Do nothing
                }
            } else {
                // False negative
                // Do nothing
            }
        }

        locate_sipc.frames++;
    } 

    void update_detect_roc(const Result & result, const GroundTruth & ground, const set<string> & templateNames, Response & response) {
        set<string> choice_labels = result.distinctLabels();
        BOOST_FOREACH(const string & subj, templateNames) {
            if (ground.onScene(subj)) {
                if (choice_labels.count(subj) == 1) {
                    response.detect_tp++;
                } else {
                    response.detect_fn++;
                }
            } else {
                if (choice_labels.count(subj) == 0) {
                    response.detect_tn++;
                } else {
                    response.detect_fp++;
                }
            }
        }
    }

    void update_locate_errors(const SetResult & resultSet,
                                const SetGroundTruth & groundSet,
                                const set<string> & /* templateNames */,
                                Response & rsp) {
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

        for (SetGroundTruth::const_iterator it = groundSet.begin(); it != groundSet.end(); it++) {
            string img_name = it->first;
            GroundTruth g = it->second;
            if (resultSet.find(img_name) == resultSet.end()) {
                throw runtime_error(str(boost::format("ERROR: No result for image '%s'") % img_name));
            }
            Result r = resultSet.find(img_name)->second;

            if (g.emptyScene()) {
                if (r.guess_made) {
                    // False positive
                    mislabelings++;
                } else {
                    // True negative
                    // Do nothing
                }
            } else {
                if (r.guess_made) {
                    if (g.onScene(r.locate_choice.getObject()->name)) {
                        // True positive
                        double a;
                        double t;
                        compute_errors(r.locate_choice, g, a, t);
                        acc_angle_err += abs(a);
                        acc_angle_sq_err += a * a;
                        acc_trans_err += abs(t);
                        acc_trans_sq_err += t * t;
                        // FIXME: these are directly parameter-dependent measures,
                        //        put max_angle_error and max_trans_error somewhere
                        if (a <= CLUTSEG_SIPC_MAX_ANGLE && t <= CLUTSEG_SIPC_MAX_TRANS) {
                            successes++;
                            acc_succ_angle_err += abs(a);
                            acc_succ_angle_sq_err += a * a;
                            acc_succ_trans_err += abs(t);
                            acc_succ_trans_sq_err += t * t;
                        }
                        
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
        }
        
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

    void ResponseFunction::operator()(const SetResult & resultSet,
                                        const SetGroundTruth & groundSet,
                                        const set<string> & templateNames,
                                        Response & rsp) {
        rsp.value = 0.0;

        rsp.detect_sipc = detect_sipc_t();
        rsp.locate_sipc = locate_sipc_t();

        for (SetGroundTruth::const_iterator it = groundSet.begin(); it != groundSet.end(); it++) {
            string img_name = it->first;
            GroundTruth g = it->second;
            if (resultSet.find(img_name) == resultSet.end()) {
                throw runtime_error(str(boost::format("ERROR: No result for image '%s'") % img_name));
            }
            Result r = resultSet.find(img_name)->second;
            update_detect_roc(r, g, templateNames, rsp);
            update_detect_sipc(r, g, templateNames, rsp.detect_sipc);
            update_locate_sipc(r, g, templateNames, rsp.locate_sipc);
        }
        
        update_locate_errors(resultSet, groundSet, templateNames, rsp);
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
                // This is support for multiple objects corresponding to the
                // very same template object.  This is not supported throughout
                // the code.
                BOOST_FOREACH(const PoseRT & truep, poses) {
                    double dt = dist_between(estp, truep); 
                    double da = angle_between(estp, truep); 
                    double r2 = (dt * dt) / (max_trans_error_ * max_trans_error_) + (da * da) / (max_angle_error_ * max_angle_error_);
                    r = r2 < r ? r2 : r;
                }
                r_acc += r;
            }
        }
        rsp.value = 1.0 - r_acc / groundSet.size();
    }

}
