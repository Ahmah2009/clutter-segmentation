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
 * Note that the SIPC scoring system does not give any scores for empty scenes.
 * Including empty scenes into the test set will render the results of SIPC
 * invalid.
 */

#ifndef _SIPC_H_
#define _SIPC_H_

#include <vector>

namespace clutseg {

    // FIXME: do proper initialization

    struct sipc_frame_t {
        sipc_frame_t() : s_h(0), s_m(0), s_n(0), s_r(0), s_t(0) {}
        /** True positives as defined by SIPC11. */
        int s_h;
        /** False negative as defined by SIPC11. In single-object case, this
         * refers to the recognizer making no choice at all. */
        int s_m;
        /** False positive as defined by SIPC11. In single-object case, this
         * refers to the recognizer estimating a pose of an object that is not
         * shown on the scene. */
        int s_n;
        /** Rotational score as defined by SIPC11. Computed for true positives
         * only. */
        float s_r;
        /** Translational score as defined by SIPC11. Computed for true
         * positives only. */
        float s_t;
    };

    struct sipc_t {
        sipc_t() : final_score(0), final_grade(0), acc_s_h(0), acc_s_m(0), acc_s_n(0), frames(0), acc_s_r(0), acc_s_t(0) {}
        /** Final score as defined by SIPC11, section A.2 */
        float final_score;
        float final_grade;
        int acc_s_h;
        int acc_s_m;
        int acc_s_n;
        int frames;
        float acc_s_r;
        float acc_s_t;
        void print();
    };

    float compute_s_r(float angle_err);
    float compute_s_t(float trans_err);

    /** Computes the combined final score for all frames. A frame corresponds
     * to what is often referred to as a scene in other parts of clutseg/XXXXX */
    sipc_t compute_sipc_score(const std::vector<sipc_frame_t> f);

}

#endif
