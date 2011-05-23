/**
 * Author: Julius Adorf
 */

#include "clutseg/result.h" 

#include <boost/foreach.hpp>

using namespace std;
using namespace tod;

namespace clutseg {

    set<string> Result::distinctLabels() const {
        set<string> s;
        BOOST_FOREACH(const Guess & g, detect_choices) {
            s.insert(g.getObject()->name);
        }
        return s;
    }

}
