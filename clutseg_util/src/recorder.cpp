
#include "recorder.h"
#include "pose_util.h"
#include <boost/foreach.hpp>
#include <opencv_candidate/PoseRT.h>

using namespace opencv_candidate;

namespace clutseg {

    TestResult::TestResult(
            const string & _imageName,
            const Features2d & _test,
            const vector<Guess> & _guesses) : 
        imageName(_imageName),
        test(_test),
        guesses(_guesses) {}


    LogRecorder::LogRecorder(const string & fname, const Options & opts) : opts_(opts) {
        log_.open(fname, FileStorage::WRITE); 
    }

    void LogRecorder::init() {
        log_ << "trainFolder" << opts_.baseDirectory;
        log_ << "test1" << "{";
        log_ << "testFolder" << opts_.imageDirectory;
        log_ << "objects" << "{";
    }

    void LogRecorder::testFinished(const TestResult & result) {
        BOOST_FOREACH(const Guess & guess, result.guesses) {
            stringstream nodeIndex;
            nodeIndex << objectIndex_;
            string nodeName = "object" + nodeIndex.str();
            log_ << nodeName << "{";
            log_ << "id" << guess.getObject()->id;
            log_ << "name" << guess.getObject()->name;
            log_ << "imageName" << result.imageName;
            log_ << PoseRT::YAML_NODE_NAME;
            // TODO: can we simplify this?
            Pose guess_pose = guess.aligned_pose();
            PoseRT guess_posert;
            poseToPoseRT(guess_pose, guess_posert);
            guess_posert.write(log_);
            log_ << "}";
            objectIndex_++;
        }
    }

    void LogRecorder::finalize() {
        log_ << "objectsCount" << (objectIndex_-1);
        log_ << "}";
        log_.release();
    }


}

