/*
 * Author: Julius Adorf
 */

/**
 * This module helps computing the score similar to the one defined in the
 * Solutions in Perception Challenge 2011 (from now on referred to as SIPC) at
 * ICRA in Shanghai. This challenge asked the contestants to correctly label
 * and locate objects in multiple test scenes. Every test scene may contain
 * more than one object.  The SIPC score is used to rank the contestants, and
 * it is a linear combination of pure classification results and pure
 * estimation results. By classifying each test scene (i.e. labeling each
 * object correctly), one can already gain 50% of the maximum score. For each
 * correctly labeled object, scores are assigned according to error of
 * estimated pose. The error is bounded though, and inversely, the score as
 * well and cannot be negative.  Bad pose estimation therefore does not
 * negatively influence final results, but locating objects precisely will give
 * you additional score. The maximum score of 100% can only be reached if all
 * objects are classified correctly and each of them is located up to error
 * margins of 3cm in translation, and 20 degrees in rotational error.
 * 
 * In the case, we only recognize one of the objects, we do not make any
 * statement about true negatives, but the SIPC score can still be calculated
 * if we make the decision that you can achieve full classificaton score by
 * always correctly labeling exactly one object in each image. Besides from
 * that, the calculation remains the same. Locating all objects is more difficult
 * than locating only one object.
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
 *  scene type          system output              n.score     ROC terminology 
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

    struct RefineSipc {
        RefineSipc() : frames(0), rscore(0), tscore(0), cscore(0) {}
        int frames;
        float rscore;
        float tscore;
        float cscore;
        float score();
        void print();
    };

    struct DetectSipc {
        DetectSipc() : objects(0), acc_score(0) {}

        int objects;
        float acc_score;
        float score();
    };

    float compute_rscore(float angle_err);
    float compute_tscore(float trans_err);

}

#endif
