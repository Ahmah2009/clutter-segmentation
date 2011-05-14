// TODO: ifdefs

/**
 * Author: Julius Adorf
 *
 * This module defines functionality for evaluating a system learnt on a
 * specific parameter set on testing set.
 */

#include <string>
#include <tod/training/feature_extraction.h>

namespace clutseg {

    std::string sha1(const std::string & file);

    std::string sha1(const tod::FeatureExtractionParams & config);

}

