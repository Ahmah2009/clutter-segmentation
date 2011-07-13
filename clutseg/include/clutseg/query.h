/*
 * Author: Julius Adorf
 */

#ifndef _QUERY_H_
#define _QUERY_H_

#include "clutseg/common.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /** \brief A query scene. */
    struct Query {
        
        Query() : img(), cloud() {}
        Query(const cv::Mat & img,
                        const PointCloudT & cloud) :
                            img(img),
                            cloud(cloud) {}
        Query(const PointCloudT & cloud) :
                            img(img),
                            cloud(cloud) {}

        cv::Mat img;
        PointCloudT cloud;

    };

}

#endif
