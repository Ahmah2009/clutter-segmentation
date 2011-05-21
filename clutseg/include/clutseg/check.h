/**
 * Author: Julius Adorf
 */

#ifndef _CHECK_H_
#define _CHECK_H_

#include <boost/filesystem.hpp>
#include <cv.h>

namespace clutseg {
    
    void assert_path_exists(const boost::filesystem::path & path);
    void assert_valid_image(const cv::Mat & img);

}

#endif
