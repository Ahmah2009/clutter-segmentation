/**
 * Author: Julius Adorf
 */

#include "clutseg/sipc.h"

#include <algorithm>
#include <boost/foreach.hpp>
#include <cmath>
#include <iostream>

using namespace std;

namespace clutseg {

    float compute_rscore(float angle_err) {
        float min_a = CLUTSEG_SIPC_MIN_ANGLE;
        float max_a = CLUTSEG_SIPC_MAX_ANGLE;
        if (angle_err > max_a) {
            return 0;
        } else if (angle_err <= min_a) {
            return 1;
        } else {
            return 1 - (angle_err - min_a) / (max_a - min_a);
        }
    }

    float compute_tscore(float trans_err) {
        float min_t = CLUTSEG_SIPC_MIN_TRANS;
        float max_t = CLUTSEG_SIPC_MAX_TRANS;
        if (trans_err > max_t) {
            return 0;
        } else if (trans_err <= min_t) {
            return 1;
        } else {
            return 1 - (trans_err - min_t) / (max_t - min_t);
        }
    }

    float locate_sipc_t::score() {
        return (0.5 * cscore + 0.25 * rscore + 0.25 * tscore) / frames;
    }

    float detect_sipc_t::score() {
        return acc_score / objects;
    }

    void locate_sipc_t::print() {
        cout << "SIPC Results" << endl
            << "---------------------------------------------------------------" << endl
            << "    Number of frames = " << frames << endl
            << endl
            << "  Recognition Score: " << endl
            << "    Total = " << cscore << endl
            << endl
            << "  Pose Score: " << endl
            << "    Rotation    = " << rscore << endl
            << "    Translation = " << tscore << endl
            << "    Total (equally weighted) = " << 0.5 * (rscore + tscore) << endl
            << endl
            << " Combined Score = " <<  0.5 * (cscore + 0.5 * (rscore + tscore)) << endl
            << " Final Score    = " << score() << endl;
    }
}
