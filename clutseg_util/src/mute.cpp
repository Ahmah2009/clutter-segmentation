/**
 * Author: Julius Adorf
 */

#include "mute.h"
#include <iostream>
#include <fstream>

using namespace std;

// see: http://www.cplusplus.com/reference/iostream/ios/rdbuf/

Mute::Mute(streambuf * null_) : null(null_) {
}

void Mute::disable() {
    backup = cout.rdbuf();
    cout.rdbuf(null);
}

void Mute::enable() {
    cout.rdbuf(backup);
}

