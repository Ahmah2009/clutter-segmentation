/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include <string>
#include <map>
#include <set>

using namespace std;

namespace clutseg {

    typedef map<string, set<string> > TestDesc;

    TestDesc loadTestDesc(const string & filename);

}

#endif
