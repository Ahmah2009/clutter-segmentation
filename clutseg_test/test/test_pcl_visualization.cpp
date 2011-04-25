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
#include <opencv2/core/eigen.hpp>
#include <pcl/registration/registration.h>

// TODO: move to own namespace
#include "rotation.h"
#include "clutseg/pcl_visualization_addons.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/registration.h>
#include <tod/training/clouds.h>
#include <tod/core/Features3d.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <tod/training/masking.h>
#include <pcl/filters/radius_outlier_removal.h>


using namespace pcl;
using namespace pcl_visualization;
using namespace cv;
using namespace opencv_candidate;
using namespace clutseg;

TEST(PclVisualization, ShowPointCloud) {
    // TODO: Getting X error when not calling visualizer.spin()
    if (enableUI) {
        PointCloud<PointXYZ> cloud;
        PointCloud<PointXYZ> cloud2;
        io::loadPCDFile("./data/sample.delimited.pcd", cloud);
        io::loadPCDFile("./data/cloud_00000.pcd", cloud2);
        PCLVisualizer visualizer;
        visualizer.addPointCloud(cloud, "cloud1");
        visualizer.addPointCloud(cloud2, "cloud2");
        visualizer.addCoordinateSystem(1, 0, 0, 0);
        visualizer.spin();
    }
}

TEST(PclVisualization, ShowSphere) {
    // TODO: Getting X error when not calling visualizer.spin()
    if (enableUI) {
        PCLVisualizer visualizer;
        PointXYZ origin;
        origin.x = 0;
        origin.y = 0;
        origin.z = 0;
        visualizer.addSphere(origin, 1, 0, 0, 1, "sphere1");
        visualizer.addSphere(origin, 2, 0, 1, 0, "sphere2");
        visualizer.addSphere(origin, 3, 1, 0, 0, "sphere3");
        visualizer.addCoordinateSystem(1, 0, 0, 0);
        visualizer.spin();
    }
}

TEST(PclVisualization, ShowPoseEstimate) {
    // TODO: Getting X error requires call to visualizer.spin()
    if (enableUI) {
        PCLVisualizer visualizer;
        PointXYZ tvec;
        tvec.x = 1;
        tvec.y = -0.25;
        tvec.z = -0.1;
        PointXYZ rvec;
        rvec.x = 1;
        rvec.y = -0.25;
        rvec.z = -0.1;
        PointXYZ origin;
        origin.x = 0;
        origin.y = 0;
        origin.z = 0;
        visualizer.addCoordinateSystem(1, 0, 0, 0);
        visualizer.addCoordinateSystem(0.2, tvec.x, tvec.y, tvec.z);
        visualizer.addLine(origin, tvec, 0.5, 0.5, 1);
        PointCloud<PointXYZ> cloud;
        io::loadPCDFile("./data/sample.delimited.pcd", cloud);
        visualizer.addPointCloud(cloud);
        visualizer.spin();
    }
}

TEST(Pcl, TransformPointCloud) {
    FileStorage in("./data/fat_free_milk_image_00000.png.pose.yaml", FileStorage::READ);
    PoseRT pose;
    pose.read(in[PoseRT::YAML_NODE_NAME]);
    in.release();

    Eigen::Matrix<float, 3, 3> R;
    Eigen::Vector3f t;
    cv::cv2eigen(pose.tvec, t);
    cv::Mat cvR;
    cv::Rodrigues(pose.rvec, cvR);
    cv::cv2eigen(cvR, R);

    Eigen::Affine3f transform;
    transform = Eigen::AngleAxisf(R.transpose());
    transform *= Eigen::Translation3f(-t);

    PointCloud<PointXYZRGB> cloud_src;
    io::loadPCDFile("./data/fat_free_milk_cloud_00000.pcd", cloud_src);
    PointCloud<PointXYZRGB> cloud_dst;
    pcl::transformPointCloud(cloud_src, cloud_dst, transform);

    if (enableUI) {
        PCLVisualizer visualizer;
        visualizer.addPointCloud(cloud_src, "cloud_src"); 
        visualizer.addPointCloud(cloud_dst, "cloud_dst"); 
        visualizer.addCoordinateSystem(0.5, 0, 0, 0);
        addPose(visualizer, pose);
        visualizer.spin();
    }
}

