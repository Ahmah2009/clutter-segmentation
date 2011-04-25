/*
 * Author: Julius Adorf
 */

#ifndef _CONN_COMP_H_
#define _CONN_COMP_H_

#include <tod/detecting/Parameters.h>

using namespace std;

namespace clutseg {

    struct Options {
        string imageDirectory;
        string baseDirectory;
        string config;
        string testdescFilename;
        string resultFilename;
        string statsFilename;
        string rocFilename;
        string tableFilename;
        string storeDirectory;
        tod::TODParameters params;
        int verbose;
        int mode;
    };

    int options(int ac, char **av, Options & opts);

}

#endif
