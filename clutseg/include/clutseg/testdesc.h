/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include <string>
#include <map>
#include <set>

namespace clutseg {

    typedef std::map<std::string, std::set<std::string> > TestDesc;

    TestDesc loadTestDesc(const std::string & filename);

}

#endif
