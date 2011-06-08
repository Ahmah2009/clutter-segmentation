/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include "clutseg/pose.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <map>
    #include <opencv_candidate/PoseRT.h>
    #include <string>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    typedef std::map<std::string, LabelSet > SetGroundTruth;

    SetGroundTruth  loadSetGroundTruth(const boost::filesystem::path & filename);

    SetGroundTruth  loadSetGroundTruthWithoutPoses(const boost::filesystem::path & filename);

}

#endif
