/*
 * Author: Julius Adorf
 */

#ifndef _PCL_VISUALIZATION_ADDONS_H_
#define _PCL_VISUALIZATION_ADDONS_H_

#include <opencv_candidate/PoseRT.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl_visualization;
using namespace pcl;
using namespace opencv_candidate;

namespace clutseg {

    void addPose(PCLVisualizer & visualizer, const PoseRT & pose, const string & id_prefix = string(""));
    void addMarker3d(PCLVisualizer & visualizer, const PointXYZ & p, const string & id_prefix = string("")); 

}

#endif
