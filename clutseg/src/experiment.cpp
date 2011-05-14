/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include <boost/format.hpp>
#include <ctype.h>
#include <iostream>
#include <stdio.h>
#include <string>

using namespace std;
using namespace tod;

namespace clutseg {

    string sha1(const string & file) {
        // http://www.gnu.org/s/hello/manual/libc/Pipe-to-a-Subprocess.html
        // http://www.gnu.org/s/hello/manual/libc/Line-Input.html#Line-Input
        FILE *in;
        in = popen(str(boost::format("sha1sum %s") % file).c_str(), "r");
        char *line = NULL;
        size_t n = 0;
        ssize_t len = getline(&line, &n, in);
        pclose(in);
        stringstream sha1;
        for (ssize_t i = 0; i < len && !isspace(line[i]); i++) {
            sha1 << line[i];
        }
        return sha1.str();
    }

    string sha1(const FeatureExtractionParams & config) {
        return "";
    }

}

