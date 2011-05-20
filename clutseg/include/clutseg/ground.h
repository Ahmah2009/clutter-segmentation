/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include <boost/filesystem.hpp>
#include <map>
#include <string>
#include <opencv_candidate/PoseRT.h>

namespace clutseg {

    struct NamedPose {

        NamedPose(const std::string & name) : name(name) {}
        NamedPose(const std::string & name, const opencv_candidate::PoseRT & pose) : name(name), pose(pose) {}

        std::string name;
        /** The pose of the object. Check pose.estimated whether it is available. */
        opencv_candidate::PoseRT pose; 

    };

    struct GroundTruth {

        std::vector<NamedPose> labels;

        bool emptyScene() const { return labels.empty(); }

        void read(const boost::filesystem::path & filename);

        bool onScene(const std::string & name) const;

    };

    typedef std::map<std::string, GroundTruth > SetGroundTruth;

    SetGroundTruth loadSetGroundTruth(const boost::filesystem::path & filename);

    SetGroundTruth loadSetGroundTruthWithoutPoses(const boost::filesystem::path & filename);

}

#endif
