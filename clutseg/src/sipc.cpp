/**
 * Author: Julius Adorf
 */

#include "clutseg/sipc.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <iostream>

using namespace std;

namespace clutseg {

    float compute_s_r(float angle_err) {
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

    float compute_s_t(float trans_err) {
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

    sipc_t compute_sipc_score(const vector<sipc_frame_t> fscores) {
        sipc_t ts;
        BOOST_FOREACH(const sipc_frame_t & f, fscores) {
            float fs = f.s_h - 0.5*f.s_m - f.s_n + (f.s_h == 1 ? 0.5*f.s_r + 0.5*f.s_t : 0);
            ts.final_score += fs > 0 ? fs : 0;
            ts.acc_s_h += f.s_h;
            ts.acc_s_m += f.s_m;
            ts.acc_s_n += f.s_n;
            ts.acc_s_r += f.s_r;
            ts.acc_s_t += f.s_t;
        }
        ts.frames = fscores.size();
        ts.final_score /= 2;
        ts.final_grade = ts.final_score / ts.frames;
        return ts;
    }

    void sipc_t::print() {
        cout << "SIPC Results" << endl
            << "---------------------------------------------------------------" << endl
            << "    Number of frames = " << frames << endl
            << "    Hits   (true positive)  = " << acc_s_h << endl
            << "    Misses (false negative) = " << acc_s_m << endl
            << "    Noise  (false positive) = " << acc_s_n << endl
            << endl
            << "  Recognition Score: (max possible score = " << float(frames) << ")" << endl
            << "    Hits   (true positive)  = " << float(acc_s_h) << endl
            << "    Misses (false negative) = " << float(-0.5 * acc_s_m) << endl
            << "    Noise  (false positive) = " << float(- acc_s_n) << endl
            << "    Negative Frame Recognition Score = 0.0" << endl
            << "    Total = " << (acc_s_h - 0.5 * acc_s_m - acc_s_n) << endl
            << endl
            << "  Pose Score: (max possible weighted score = " << float(frames) << ")" << endl
            << "    Rotation    = " << float(acc_s_r) << endl
            << "    Translation = " << float(acc_s_t) << endl
            << "    Total (equally weighted) = " << float(0.5 * acc_s_r + acc_s_t) << endl
            << endl
            << " Combined Score = " << 0.5 * (0.5 * acc_s_r + 0.5 * acc_s_t + acc_s_h - 0.5 * acc_s_m - acc_s_n) << endl
            << " Final Score    = " << final_score << endl
            << " Final Grade = " << final_grade << endl
            << " Time Duration >= UNKNOWN" << endl;
    }
}
