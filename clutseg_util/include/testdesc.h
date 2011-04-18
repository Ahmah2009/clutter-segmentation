/*
 * Author: Julius Adorf
 */

#include <string>
#include <map>
#include <set>

using namespace std;

namespace clutseg {

    typedef map<string, set<string> > TestDesc;

    TestDesc loadTestDesc(const string & filename);

}

