/* 
 * Author: Julius Adorf
 */

#include "test.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_candidate/PoseRT.h>
#include <boost/format.hpp>
#include <tod/detecting/Tools.h>

#include <pcl_visualization_addons.h>

using namespace cv;
using namespace pcl;
using namespace opencv_candidate;
using namespace clutseg;

// ||rvec|| is the angle, and rvec is the axis of rotation
TEST(PclVisualization, ShowFiducialPoseEstimate) {
    // TODO: Getting X error when not calling visualizer.spin()
    if (enableUI) {
        // Load pose estimation from yaml file
        FileStorage fs("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
        PoseRT pose;
        pose.read(fs[PoseRT::YAML_NODE_NAME]);
        // Load point cloud
        PointCloud<PointXYZ> cloud;
        io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
        // Create visualizer
        PCLVisualizer visualizer;
        // Add coordinate system
        visualizer.addCoordinateSystem(0.5, 0, 0, 0);
        // Add point cloud
        visualizer.addPointCloud(cloud);
        // Draw pose
        addPose(visualizer, pose);
        visualizer.spin();
    }
}

TEST(PclVisualization, ShowInvertedPose) {
    // TODO: Getting X error when not calling visualizer.spin()
    if (enableUI) {
        // Load pose estimation from yaml file
        FileStorage fs("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
        PoseRT pose;
        pose.read(fs[PoseRT::YAML_NODE_NAME]);
        PoseRT invPose = tod::Tools::invert(pose);
        // Load point cloud
        PointCloud<PointXYZ> cloud;
        io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud);
        // Create visualizer
        PCLVisualizer visualizer;
        // Add coordinate system
        visualizer.addCoordinateSystem(0.5, 0, 0, 0);
        // Add point cloud
        visualizer.addPointCloud(cloud);
        // Draw pose
        addPose(visualizer, pose, "standard");
        // Draw pose
        addPose(visualizer, invPose, "inverted");
        visualizer.spin();
    }
}


TEST(PclVisualization, ShowAllPoses) {
    // TODO: Getting X error when not calling visualizer.spin()
    if (enableUI) {
        PCLVisualizer visualizer;
        visualizer.addCoordinateSystem(0.5, 0, 0, 0);
        PoseRT pose;
        for (int i = 0; i < 76; i++) {
            FileStorage fs(str(boost::format("./data/pose/image_%05i.png.pose.yaml") % i), FileStorage::READ);
            pose.read(fs[PoseRT::YAML_NODE_NAME]);
            addPose(visualizer, pose, str(boost::format("pose-%05i-") % i));
        }    
        visualizer.spin();
    }
}

