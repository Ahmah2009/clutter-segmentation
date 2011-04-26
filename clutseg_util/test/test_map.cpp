/*
 * Author: Julius Adorf
 */

#include "test.h"
 #include <gtest/gtest.h>
#include "clutseg/pcl_visualization_addons.h"
#include "clutseg/map.h"
#include <cv.h>
#include <boost/format.hpp>

#include <pcl_visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace pcl_visualization;
using namespace clutseg;

class Map : public ::testing::Test {
    public:
        virtual void SetUp() {
            sampleColorImage(img);
            sampleCloud(cloud);
            samplePose(pose);
        }

        Mat img;
        PoseRT pose;
        PointCloud<PointXYZ> cloud;
};

// TODO: move to test_pcl_visualization_addons
TEST_F(Map, AddMarker3d) {
    PCLVisualizer vis;
    PointXYZ p(pose.tvec.at<double>(0, 0), pose.tvec.at<double>(1, 0), pose.tvec.at<double>(2, 0));
    addMarker3d(vis, p);
    vis.addPointCloud(cloud);
    vis.spin();
}

TEST_F(Map, MapImageCorners) {
    vector<Point> corners;
    corners.push_back(Point(0, 0));
    corners.push_back(Point(img.cols - 1, 0));
    corners.push_back(Point(img.cols - 1, img.rows - 1));
    corners.push_back(Point(0, img.rows - 1));
    PointCloud<PointXYZ> corners3d;
    mapToCloud(corners3d, corners, img, cloud); 
    EXPECT_EQ(corners.size(), corners3d.size());
    EXPECT_EQ(4, corners.size());
    EXPECT_EQ(4, corners3d.size());
    for (PointCloud<PointXYZ>::iterator it = corners3d.begin(),
         end = corners3d.end(); it != end; it++) {
        cout << boost::format("%d: %8f,%8f,%8f") % (it - corners3d.begin()) % (*it).x % (*it).y % (*it).z << endl;
    }
    // show the whole stuff
    PCLVisualizer vis;
    addPose(vis, PoseRT());
    addMarker3d(vis, corners3d);
    vis.spin();
}

