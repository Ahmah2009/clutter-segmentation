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
 * Challenge)", in the following referred to as SIPC11 for a description. The
 * report is not always precise and instead one can look at the source code
 * that implements the score at
 * http://opencv.willowgarage.com/wiki/SolutionsInPerceptionChallenge?action=AttachFile&do=view&target=GroundTruth_And_Scripts.tgz
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
 *
 * These test bags do not show any scene where two instances are visible that
 * belong to the same template object. Yet, the SIPC documentation (2011-05-26)
 * says it is possible that "duplicate objects" might appear in several poses on
 * the very same test scene. Also, the source code for computing these scores
 * has a matching routine that associates detected objects with ground truth
 * objects. The most sensible approach is here to match instances to the closest
 * ground truth instances, and in case the number of detected duplicates does not
 * match the number of ground truth objects, there are either false negatives, or
 * false positives.
 *
 * The guys from the Challenge haven't answered to my questions yet, and I was
 * unable to obtain detailed reports about the contestants' performance. I am
 * therefore very much not inclined to either use their code for scoring. A
 * simple escape would be to just not care about duplicates. That's a
 * simplification, and having prior knowledge about the test data leads to bias
 * of the classifier/estimator (e.g. can discard duplicate guesses right away
 * to improve SIPC score, if we knew in advance whether test scenes will
 * contain duplicates or not). For sake of simplicity, I take this shortcut,
 * but will document this decision.
 *
 * See 
 * http://vault.willowgarage.com/wgdata1/vol1/solutions_in_perception/Willow_Final_Test_Set/tests.zip
 * for some test data.
 */

#ifndef _SIPC_H_
#define _SIPC_H_

#include <vector>

#define CLUTSEG_SIPC_MIN_TRANS 0.01
#define CLUTSEG_SIPC_MAX_TRANS 0.03
#define CLUTSEG_SIPC_MIN_ANGLE (M_PI/90)
#define CLUTSEG_SIPC_MAX_ANGLE (M_PI/9)

namespace clutseg {

    struct locate_sipc_t {
        locate_sipc_t() : frames(0), rscore(0), tscore(0), cscore(0) {}
        int frames;
        float rscore;
        float tscore;
        float cscore;
        float score();
        void print();
    };

    struct detect_sipc_t {
        detect_sipc_t() : objects(0), acc_score(0) {}

        int objects;
        float acc_score;
        float score();
    };

    float compute_rscore(float angle_err);
    float compute_tscore(float trans_err);

}

#endif
