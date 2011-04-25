/*
 * Author: Julius Adorf
 */

 #include <gtest/gtest.h>
#include "pcl_visualization_addons.h"
#include "test.h"
#include <cv.h>

//#include "clutseg/map.h"
//#include <pcl_visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
using namespace clutseg;

class Map : public ::testing::Test {
    public:
        virtual void SetUp() {
            sampleColorImage(queryImage);
            sampleCloud(queryCloud);
            samplePose(pose);
        }

        Mat queryImage;
        PoseRT pose;
        PointCloud<PointXYZ> queryCloud;
};
// TODO: move to test_pcl_visualization_addons
TEST_F(Map, AddMarker3d) {
    PCLVisualizer vis;
    PointXYZ p(pose.tvec.at<double>(0, 0), pose.tvec.at<double>(1, 0), pose.tvec.at<double>(2, 0));
    addMarker3d(vis, p);
    vis.addPointCloud(queryCloud);
    vis.spin();
}
