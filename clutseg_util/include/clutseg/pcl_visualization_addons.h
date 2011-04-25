/*
 * Author: Julius Adorf
 */

#ifndef _PCL_VISUALIZATION_ADDONS_H_
#define _PCL_VISUALIZATION_ADDONS_H_

#include <opencv_candidate/PoseRT.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

namespace clutseg {

    void addPose(pcl_visualization::PCLVisualizer & visualizer,
                const opencv_candidate::PoseRT & pose,
                const std::string & id_prefix = std::string(""));

    void addMarker3d(pcl_visualization::PCLVisualizer & visualizer,
                    const pcl::PointXYZ & p,
                    const std::string & id_prefix = std::string("")); 

}

#endif
