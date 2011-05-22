/**
 * Author: Julius Adorf
 *
 * This module helps computing the score as defined by Solutions in Perception
 * Challenge 2011 (from now on referred to as SIPC) at ICRA in Shanghai. There
 * is a notable difference between the SIPC score and this score since we try
 * only to recognize one object. The score calculation remains the same. It is
 * true that the scores cannot be directly compared to the contestants' scores.
 * Yet, the recognizer that tries only to locate one object can be regarded as
 * a classifier/estimator with a priori information about all but one object in
 * the scene.
 *
 * See "How To Read a Detailed Score Report (ICRA2011 Solutions in Perception
 * Challenge)", in the following referred to as SIPC11 for a description.
 *
 * Note that the SIPC scoring system does not give any scores for empty scenes,
 * and I extended the scoring system to cover it.
 *
 * The following cases can happen in a test scene:
 *
 *  scene type          choice                      score       ROC terminology 
 * ----------------------------------------------------------------------------
 *   empty              none                        1.0         true negative
 *   empty              some object not on scene    0.0         false positive    
 *   not empty          none                        0.0         false negative
 *   not empty          some object on scene        0.5 + x     true positive
 *   not empty          some object not on scene    0.0         false positive
 */

#ifndef _SIPC_H_
#define _SIPC_H_

#include <vector>

namespace clutseg {

    struct sipc_t {
        sipc_t() : final_score(0), acc_tp(0), acc_fp(0), acc_tn(0), acc_fn(0),
                    frames(0), acc_rscore(0), acc_tscore(0), acc_cscore(0),
                    max_cscore(0), max_rscore(0), max_tscore(0) {}
        float final_score;
        int acc_tp;
        int acc_fp;
        int acc_tn;
        int acc_fn;
        int frames;
        float acc_rscore;
        float acc_tscore;
        float acc_cscore;
        float max_cscore;
        float max_rscore;
        float max_tscore;
        void compute_final_score();
        void print();
    };

    float compute_s_r(float angle_err);
    float compute_s_t(float trans_err);

}

#endif
