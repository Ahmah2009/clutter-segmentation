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
        float min_a = M_PI / 90;
        float max_a = M_PI / 9;
        if (angle_err > max_a) {
            return 0;
        } else if (angle_err <= min_a) {
            return 1;
        } else {
            return 1 - (angle_err - min_a) / (max_a - min_a);
        }
    }

    float compute_tscore(float trans_err) {
        float min_t = 0.01;
        float max_t = 0.03;
        if (trans_err > max_t) {
            return 0;
        } else if (trans_err <= min_t) {
            return 1;
        } else {
            return 1 - (trans_err - min_t) / (max_t - min_t);
        }
    }

    void sipc_t::compute_final_score() {
        final_score = (0.5 * cscore + 0.25 * rscore + 0.25 * tscore) / frames;
    }

    void sipc_t::print() {
        cout << "SIPC Results" << endl
            << "---------------------------------------------------------------" << endl
            << "    Number of frames = " << frames << endl
        //    << "    Hits   (true positive)  = " << tp << endl
        //    << "    Misses (false negative) = " << fn << endl
        //    << "    Noise  (false positive) = " << fp << endl
        //    << "    Correct rejections (true negative)) = " << tn << endl
            << endl
            << "  Recognition Score: (max possible score = " << float(max_cscore) << ")" << endl
            << "    Total = " << cscore << endl
            << endl
            << "  Pose Score: (max possible weighted score = " << float(0.5 * max_rscore + 0.5 * max_tscore) << ")" << endl
            << "    Rotation    = " << rscore << endl
            << "    Translation = " << tscore << endl
            << "    Total (equally weighted) = " << 0.5 * (rscore + tscore) << endl
            << endl
            << " Combined Score = " <<  0.5 * (cscore + 0.5 * (rscore + tscore)) << endl
            << " Final Score    = " << final_score << endl;
        //    << " Time Duration >= UNKNOWN" << endl;
    }
}
