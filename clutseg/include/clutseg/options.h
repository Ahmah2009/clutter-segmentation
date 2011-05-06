/*
 * Author: Julius Adorf
 */

#ifndef _CONN_COMP_H_
#define _CONN_COMP_H_

#include <tod/detecting/Parameters.h>
#include <string>

namespace clutseg {

    struct Options {
        std::string imageDirectory;
        std::string baseDirectory;
        std::string config;
        std::string testdescFilename;
        std::string resultFilename;
        std::string statsFilename;
        std::string rocFilename;
        std::string tableFilename;
        std::string storeDirectory;
        tod::TODParameters params;
        int verbose;
        int mode;
    };

    int options(int ac, char **av, Options & opts);

}

#endif
