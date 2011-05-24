/**
 * Author: Julius Adorf
 */

#include "clutseg/report.h"

#include "clutseg/pose.h"

#include <cassert>

using namespace cv;
using namespace opencv_candidate;
using namespace std;

namespace clutseg {

    float TestReport::angle_error() const {
        assert(result.guess_made);
        vector<PoseRT> poses = ground.posesOf(result.locate_choice.getObject()->name);
        assert(poses.size() == 1);
        PoseRT truep = poses[0];
        Pose estp = result.locate_choice.aligned_pose();
        return angle_between(estp, truep); 
 
    }

    float TestReport::trans_error() const {
        assert(result.guess_made);
        vector<PoseRT> poses = ground.posesOf(result.locate_choice.getObject()->name);
        assert(poses.size() == 1);
        PoseRT truep = poses[0];
        Pose estp = result.locate_choice.aligned_pose();
        return dist_between(estp, truep); 
    }

    bool TestReport::success() const {
        return result.guess_made && (angle_error() <= M_PI / 9 && trans_error() <= 0.03);
    }

}
