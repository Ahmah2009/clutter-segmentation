/*
 * Author: Julius Adorf
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
#include "clutseg/gcc_diagnostic_enable.h"

// see (pending?)
// http://stackoverflow.com/questions/6227420/how-to-use-gcc-diagnostic-pragma-with-c-template-functions
#pragma GCC diagnostic ignored "-Wunused-parameter"
namespace clutseg {

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; 

}

#endif
