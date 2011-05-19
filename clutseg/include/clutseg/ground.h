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

    typedef std::vector<NamedPose> GroundTruth;

    typedef std::map<std::string, GroundTruth > TestSetGroundTruth;

    TestSetGroundTruth loadTestSetGroundTruth(const boost::filesystem::path & filename);

    TestSetGroundTruth loadTestSetGroundTruthWithoutPoses(const boost::filesystem::path & filename);

}

#endif
