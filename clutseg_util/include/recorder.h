/* draft */

#include "options.h"
#include <tod/detecting/GuessGenerator.h>
#include <cv.h>
#include <vector>

using namespace std;
using namespace tod;
using namespace cv;

namespace clutseg {

    struct TestResult {
        TestResult(
            const string & _imageName,
            const Features2d & _test,
            const vector<Guess> & _guesses);
        const string & imageName;
        const Features2d & test;
        const vector<Guess> & guesses;
    };

    class Recorder {
        public:
            virtual void init() = 0;
            virtual void testFinished(const TestResult & result) = 0;
            virtual void finalize() = 0;
    };

    class LogRecorder : public Recorder {
        public:
            LogRecorder(const string & fname, const Options & opts);
            virtual void init();
            virtual void testFinished(const TestResult & result);
            virtual void finalize();
        private:
            const Options & opts_;
            FileStorage log_;
            int objectIndex_;
    };

}

