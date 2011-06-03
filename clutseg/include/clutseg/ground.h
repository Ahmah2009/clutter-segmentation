/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <map>
    #include <string>
    #include <opencv_candidate/PoseRT.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    struct LabeledPose {

        LabeledPose() : name("") {}
        LabeledPose(const std::string & name) : name(name) {}
        LabeledPose(const std::string & name, const opencv_candidate::PoseRT & pose) : name(name), pose(pose) {}

        std::string name;
        /** The pose of the object. Check pose.estimated whether it is available. */
        opencv_candidate::PoseRT pose; 

    };

    struct GroundTruth {

        std::vector<LabeledPose> labels;

        bool emptyScene() const { return labels.empty(); }
        bool onScene(const std::string & name) const;
        int distinctLabelCount() const;
        std::vector<opencv_candidate::PoseRT> posesOf(const std::string & subject) const;

        void read(const boost::filesystem::path & filename);
        void write(const boost::filesystem::path & filename);

    };

    typedef std::map<std::string, GroundTruth > SetGroundTruth;

    SetGroundTruth loadSetGroundTruth(const boost::filesystem::path & filename);

    SetGroundTruth loadSetGroundTruthWithoutPoses(const boost::filesystem::path & filename);

}

#endif
