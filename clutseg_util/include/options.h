
#include <tod/detecting/Parameters.h>

using namespace std;
using namespace tod;

namespace clutseg {

    struct Options {
        string imageDirectory;
        string baseDirectory;
        string config;
        string testdescFilename;
        string resultFilename;
        string statsFilename;
        string logFilename;
        string rocFilename;
        string tableFilename;
        string storeDirectory;
        TODParameters params;
        int verbose;
        int mode;
    };

}

