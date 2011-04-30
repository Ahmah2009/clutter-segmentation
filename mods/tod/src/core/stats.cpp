#include "tod/core/stats.h"

#include <boost/format.hpp>

using namespace std;
using namespace boost;
        
posest_stats::posest_stats() : 
    time(0.0), img_cnt(0), success_cnt(0), failure_cnt(0),
    orig_success_cnt(0), orig_failure_cnt(0), fallback_success_cnt(0),
    fallback_failure_cnt(0) { }

void posest_stats::print(ostream & out) const {
    // TODO: use YAML persistence module
    out << "%YAML:1.0" << endl;
    out << "posest_stats" << " {" << endl;
    out << format("    %-22s: %5.1f") % "time" % time << endl;
    out << format("    %-22s: %5.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %5d") % "img_cnt" % img_cnt << endl;
    out << format("    %-22s: %5d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %5d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %5.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %5.2f") % "failure_rate" % failure_rate() << endl;
    out << format("    %-22s: %5d") % "orig_success_cnt" % orig_success_cnt << endl;
    out << format("    %-22s: %5d") % "orig_failure_cnt" % orig_failure_cnt << endl;
    out << format("    %-22s: %5.2f") % "orig_success_rate" % orig_success_rate() << endl;
    out << format("    %-22s: %5.2f") % "orig_failure_rate" % orig_failure_rate() << endl;
    out << format("    %-22s: %5d") % "fallback_success_cnt" % fallback_success_cnt  << endl;
    out << format("    %-22s: %5d") % "fallback_failure_cnt" % fallback_failure_cnt  << endl;
    out << format("    %-22s: %5.2f") % "fallback_success_rate" % fallback_success_rate()  << endl;
    out << format("    %-22s: %5.2f") % "fallback_failure_rate" % fallback_failure_rate()  << endl;
    out << "}" << endl;
}

masker_stats::masker_stats() : 
    time(0.0), img_cnt(0), success_cnt(0), failure_cnt(0) { }

void masker_stats::print(ostream & out) const {
    out << "%YAML:1.0" << endl;
    out << "masker_stats" << " {" << endl;
    out << format("    %-22s: %5.1f") % "time" % time << endl;
    out << format("    %-22s: %5.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %5d") % "img_cnt" % img_cnt << endl;
    out << format("    %-22s: %5d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %5d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %5.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %5.2f") % "failure_rate" % failure_rate() << endl;
    out << "}" << endl;
}

detector_stats::detector_stats() : 
    time(0.0), img_cnt(0), success_cnt(0), failure_cnt(0), tot_keypoint_cnt(0),
    min_keypoint_cnt(numeric_limits<int>::max()),
    max_keypoint_cnt(numeric_limits<int>::min()), pm_threshold_used(true),
    pm_min_features_used(true), pm_max_features_used(true),
    pm_octaves_used(true)  { }

void detector_stats::print(ostream & out) const {
    out << "%YAML:1.0" << endl;
    out << "detector_stats" << " {" << endl;
    out << format("    %-22s: %5.1f") % "time" % time << endl;
    out << format("    %-22s: %5.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %5d") % "img_cnt" % img_cnt << endl;
    out << endl;
    out << format("    %-22s: %5d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %5d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %5.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %5.2f") % "failure_rate" % failure_rate() << endl;
    out << endl;
    out << format("    %-22s: %5d") % "tot_keypoint_cnt" % tot_keypoint_cnt << endl;
    out << format("    %-22s: %5d") % "avg_keypoint_cnt" % int(avg_keypoint_cnt()) << endl;
    out << format("    %-22s: %5d") % "min_keypoint_cnt" % min_keypoint_cnt << endl;
    out << format("    %-22s: %5d") % "max_keypoint_cnt" % max_keypoint_cnt << endl;
    out << endl;
    out << format("    %-22s: %5s") % "pm_threshold_used" % pm_threshold_used << endl;
    out << format("    %-22s: %5s") % "pm_min_features_used" % pm_min_features_used << endl;
    out << format("    %-22s: %5s") % "pm_max_features_used" % pm_max_features_used << endl;
    out << format("    %-22s: %5s") % "pm_octaves_used" % pm_octaves_used << endl;
    /* out << endl; // TODO: include feature extraction params
    out << format("    %-22s: %5s") % "detector_type" % params.detector_type << endl;
    out << format("    %-22s: %5s") % "descriptor_type" % params.descriptor_type << endl;
    out << format("    %-22s: %5s") % "extractor_type" % params.extractor_type << endl; */
    // TODO: include detector_params and extractor_params
    out << endl;
    out << "}" << endl;
}

