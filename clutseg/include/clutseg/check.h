/*
 * Author: Julius Adorf
 */

#ifndef _CHECK_H_
#define _CHECK_H_

#include <boost/filesystem.hpp>
#include <cv.h>

namespace clutseg {
    
    /** \brief Assertion. Causes a runtime error in case the path does not exist. */
    void assert_path_exists(const boost::filesystem::path & path);

    /** \brief Assertion. Causes a runtime error in case the image is empty. */
    void assert_valid_image(const cv::Mat & img);

}

#endif
