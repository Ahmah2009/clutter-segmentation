/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <ctype.h>
#include <cv.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <string>

using namespace cv;
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
        stringstream s;
        for (ssize_t i = 0; i < len && !isspace(line[i]); i++) {
            s << line[i];
        }
        return s.str();
    }

    string sha1(const FeatureExtractionParams & config) {
        char buffer[L_tmpnam];
        char *ignoreme = tmpnam(buffer);
        stringstream fn;
        fn << buffer;
        FileStorage fs(fn.str(), FileStorage::WRITE);
        fs << FeatureExtractionParams::YAML_NODE_NAME;
        config.write(fs);
        fs.release();
        string s = sha1(fn.str()); 
        boost::filesystem::remove(fn.str());
        return s;
    }

}

