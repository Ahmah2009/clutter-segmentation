/**
 * Author: Julius Adorf
 */

#include <stdlib.h>
#include <iostream>
#include "misc.h"

using namespace std;

int extractAngleFromFileName(const string & filename) {
    int s = filename.find("_") + 1;
    int e = filename.rfind("_");
    std::cout << filename.substr(s, e - s) << std::endl;
    return atoi(filename.substr(s, e - s).c_str());
}

