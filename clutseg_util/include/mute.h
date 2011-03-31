/**
 * Author: Julius Adorf
 */

#include <iostream>
#include <ostream>

using namespace std;

/** Tries to mute standard out. Unfortunately, when threading is involved
 * that does no longer work.
 */
class Mute {
    public:
        Mute(streambuf * null);
        void disable();
        void enable();
    private:
        streambuf * backup;
        streambuf * null;
};

