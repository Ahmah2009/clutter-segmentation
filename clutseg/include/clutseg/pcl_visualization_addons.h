/*
 * Author: Julius Adorf
 */

#ifndef _PCL_VISUALIZATION_ADDONS_H_
#define _PCL_VISUALIZATION_ADDONS_H_

#include "clutseg/common.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <opencv_candidate/PoseRT.h>
    #include <pcl/point_types.h>
    #include <pcl_visualization/pcl_visualizer.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /** \brief Adds the axes of a coordinate system described by a pose to the PCLVisualizer */
    void addPose(pcl_visualization::PCLVisualizer & visualizer,
                const opencv_candidate::PoseRT & pose,
                const std::string & id_prefix = std::string(""));

    /** \brief Add a marker to the PCLVisualizer at a specific point. */
    void addMarker3d(pcl_visualization::PCLVisualizer & visualizer,
                    const pcl::PointXYZ & p,
                    int r = 255, int g = 0, int b = 0, 
                    const std::string & id_prefix = std::string("")); 

    /** \brief Add multiple markers to the PCLVisualizer */
    void addMarker3d(pcl_visualization::PCLVisualizer & visualizer,
                    const PointCloudT & cloud,
                    int r = 255, int g = 0, int b = 0, 
                    const std::string & id_prefix = std::string("")); 

    /** \brief Add a polygon to the PCLVisualizer.
     *
     * The point cloud is interpreted as an ordered list of polygon vertices.
     */
    void addMarkerPolygon3d(pcl_visualization::PCLVisualizer & visualizer,
                    const PointCloudT & cloud,
                    int r = 255, int g = 0, int b = 0,
                    const std::string & id_prefix = std::string("")); 


}

#endif
