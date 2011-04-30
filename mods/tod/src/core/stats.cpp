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
    out << format("    %-22s: %4.1f") % "time" % time << endl;
    out << format("    %-22s: %4.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %4d") % "img_cnt" % img_cnt << endl;
    out << format("    %-22s: %4d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %4d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %4.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %4.2f") % "failure_rate" % failure_rate() << endl;
    out << format("    %-22s: %4d") % "orig_success_cnt" % orig_success_cnt << endl;
    out << format("    %-22s: %4d") % "orig_failure_cnt" % orig_failure_cnt << endl;
    out << format("    %-22s: %4.2f") % "orig_success_rate" % orig_success_rate() << endl;
    out << format("    %-22s: %4.2f") % "orig_failure_rate" % orig_failure_rate() << endl;
    out << format("    %-22s: %4d") % "fallback_success_cnt" % fallback_success_cnt  << endl;
    out << format("    %-22s: %4d") % "fallback_failure_cnt" % fallback_failure_cnt  << endl;
    out << format("    %-22s: %4.2f") % "fallback_success_rate" % fallback_success_rate()  << endl;
    out << format("    %-22s: %4.2f") % "fallback_failure_rate" % fallback_failure_rate()  << endl;
    out << "}" << endl;
}

masker_stats::masker_stats() : 
    time(0.0), img_cnt(0), success_cnt(0), failure_cnt(0) { }

void masker_stats::print(ostream & out) const {
    out << "%YAML:1.0" << endl;
    out << "masker_stats" << " {" << endl;
    out << format("    %-22s: %4.1f") % "time" % time << endl;
    out << format("    %-22s: %4.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %4d") % "img_cnt" % img_cnt << endl;
    out << format("    %-22s: %4d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %4d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %4.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %4.2f") % "failure_rate" % failure_rate() << endl;
    out << "}" << endl;
}

detector_stats::detector_stats() : 
    time(0.0), img_cnt(0), success_cnt(0), failure_cnt(0) { }

void detector_stats::print(ostream & out) const {
    out << "%YAML:1.0" << endl;
    out << "detector_stats" << " {" << endl;
    out << format("    %-22s: %4.1f") % "time" % time << endl;
    out << format("    %-22s: %4.1f") % "avg_time" % avg_time() << endl;
    out << format("    %-22s: %4d") % "img_cnt" % img_cnt << endl;
    out << format("    %-22s: %4d") % "success_cnt" % success_cnt << endl;
    out << format("    %-22s: %4d") % "failure_cnt" % failure_cnt << endl;
    out << format("    %-22s: %4.2f") % "success_rate" % success_rate() << endl;
    out << format("    %-22s: %4.2f") % "failure_rate" % failure_rate() << endl;
    out << "}" << endl;
}

