/**
 * Author: Julius Adorf
 *
 * This module helps computing the score as defined by Solutions in Perception
 * Challenge 2011 (from now on referred to as SIPC) at ICRA in Shanghai. This
 * challenge asked the contestants to correctly label and locate objects in
 * multiple test scenes. Every test scene may contain more than one object.
 * The SIPC score is used to rank the contestants, and it is a linear
 * combination of pure classification results and pure estimation results.  By
 * classifying each test scene (i.e. labeling each object correctly), one can
 * already gain 50% of the maximum score. For each correctly labeled object,
 * scores are assigned according to error of estimated pose. The error is
 * bounded though, and inversely, the score as well and cannot be negative.
 * Bad pose estimation therefore does not negatively influence final results,
 * but locating objects precisely will give you additional score. The maximum
 * score of 100% can only be reached if all objects are classified correctly
 * and each of them is located up to error margins of 3cm in translation, and
 * 20 degrees in rotational error.
 * 
 * In the case, we only recognize one of the objects, we do not make any
 * statement about true negatives, and the SIPC score can still be calculated
 * with slight modification that you can achieve full classificaton score by
 * always correctly labeling any object in each image. Besides from that the
 * calculation remains the same. The scores achieved by attempting to recognize
 * all objects, or (likely) easier cannot be compared directly. Yet, since we
 * assume that being able to choose one already labeled object for pose estimation
 * will decrease expected estimation error, and the scores on all objects shall 
 * provide a baseline for comparison.
 *
 * See "How To Read a Detailed Score Report (ICRA2011 Solutions in Perception
 * Challenge)", in the following referred to as SIPC11 for a description.
 *
 * Note that the SIPC scoring system does not give any scores for empty scenes.
 * A sensible extension would be to just assign full score for an image if it
 * has been correctly classified to not show any objects, and give zero score
 * otherwise.
 *
 * The following cases can happen in a test scene (when attempting to recognize
 * single objects only).
 *
 *  scene type          choice                      n.score     ROC terminology 
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
        sipc_t() : final_score(0), frames(0), rscore(0), tscore(0), cscore(0),
                    max_cscore(0), max_rscore(0), max_tscore(0) {}
        float final_score;
        int frames;
        float rscore;
        float tscore;
        float cscore;
        int max_cscore;
        int max_rscore;
        int max_tscore;
        void compute_final_score();
        void print();
    };

    float compute_rscore(float angle_err);
    float compute_tscore(float trans_err);

}

#endif
