/*
 * Author: Julius Adorf
 */

#ifndef _TESTDESC_H_
#define _TESTDESC_H_

#include <boost/filesystem.hpp>
#include <map>
#include <set>
#include <string>

namespace clutseg {

    typedef std::map<std::string, std::set<std::string> > TestDesc;

    TestDesc loadTestDesc(const boost::filesystem::path & filename);

}

#endif
