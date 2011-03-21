/**
 * Author: Julius Adorf
 */

#include <stdlib.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include "misc.h"

int extractAngleFromFileName(const string & filename) {
    int s = filename.find("_") + 1;
    int e = filename.rfind("_");
    std::cout << filename.substr(s, e - s) << std::endl;
    return atoi(filename.substr(s, e - s).c_str());
}

string angleToFourChars(int angle) {
    string as = boost::lexical_cast<string>(angle);
    string ns = boost::lexical_cast<string>(-angle);
    if (angle <= -100) {
        return as;
    } else if (angle <= -10) {
        return "-0" + ns;
    } else if (angle < 0) {
        return "-00" + ns;
    } else if (angle < 10) {
        return "000" + as;
    } else if (angle < 100) {
        return "00" + as;
    } else {
        return "0" + as;
    }
}

