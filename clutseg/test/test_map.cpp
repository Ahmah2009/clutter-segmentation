/*
 * Author: Julius Adorf
 */

#include "test.h"

#include <gtest/gtest.h>

#include "clutseg/common.h"
#include "clutseg/pcl_visualization_addons.h"
#include "clutseg/map.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
    #include <boost/format.hpp>
    #include <pcl_visualization/pcl_visualizer.h>
#include "clutseg/gcc_diagnostic_enable.h"

#pragma GCC diagnostic ignored "-Wunused-parameter"

using namespace std;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace pcl_visualization;
using namespace clutseg;

class test_map : public ::testing::Test {
    public:
        virtual void SetUp() {
            sampleColorImage(img);
            markerImg = imread("data/image_00000.marker.png");
            sampleCloud(cloud);
            samplePose(pose);
        }

        Mat img;
        Mat markerImg;
        PoseRT pose;
        PointCloudT cloud;
};

TEST_F(test_map, add_marker_3d) {
    SKIP_IF_FAST

    PCLVisualizer vis;
    PointXYZ p(pose.tvec.at<double>(0, 0), pose.tvec.at<double>(1, 0), pose.tvec.at<double>(2, 0));
    addMarker3d(vis, p);
    vis.addPointCloud(cloud);
    vis.spin();
}

TEST_F(test_map, map_image_corners) {
    SKIP_IF_FAST

    vector<Point> corners;
    // select points close to corners, avoid NaNs
    int margin = 160;
    corners.push_back(Point(margin, margin));
    corners.push_back(Point(img.cols - margin, margin));
    corners.push_back(Point(img.cols - margin, img.rows - margin));
    corners.push_back(Point(margin, img.rows - margin));
    PointCloudT corners3d;
    mapToCloud(corners3d, corners, img, cloud); 
    EXPECT_EQ(corners.size(), corners3d.size());
    EXPECT_EQ(4, corners.size());
    EXPECT_EQ(4, corners3d.size());
    for (PointCloudT::iterator it = corners3d.begin(),
         end = corners3d.end(); it != end; it++) {
        cout << boost::format("%d: %8f,%8f,%8f") % (it - corners3d.begin()) % (*it).x % (*it).y % (*it).z << endl;
    }
    // show the whole stuff
    PCLVisualizer vis;
    addPose(vis, PoseRT());
    addMarkerPolygon3d(vis, corners3d);
    vis.addPointCloud(cloud);
    vis.spin();
}

TEST_F(test_map, map_markers) {
    SKIP_IF_FAST

    vector<Point> red2d;
    vector<Point> green2d;
    vector<Point> yellow2d;
    vector<Point> blue2d;

    red2d.push_back(Point(727, 269));
    red2d.push_back(Point(716, 448));
    red2d.push_back(Point(806, 449));
    red2d.push_back(Point(834, 268));

    green2d.push_back(Point(336, 195));
    green2d.push_back(Point(1157, 198));

    yellow2d.push_back(Point(338, 183));
    yellow2d.push_back(Point(1154, 189));

    blue2d.push_back(Point(697, 86)); 
    blue2d.push_back(Point(371, 92)); 
    
    PointCloudT red3d;
    PointCloudT green3d;
    PointCloudT yellow3d;
    PointCloudT blue3d;

    mapToCloud(red3d, red2d, markerImg, cloud); 
    mapToCloud(green3d, green2d, markerImg, cloud); 
    mapToCloud(yellow3d, yellow2d, markerImg, cloud); 
    mapToCloud(blue3d, blue2d, markerImg, cloud); 
   
    /* 
    PointCloudT red3d_hscaled;
    PointCloudT green3d_hscaled;
    PointCloudT yellow3d_hscaled;
    PointCloudT blue3d_hscaled;

    mapToCloud(red3d_hscaled, red2d, markerImg, cloud, false); 
    mapToCloud(green3d_hscaled, green2d, markerImg, cloud, false); 
    mapToCloud(yellow3d_hscaled, yellow2d, markerImg, cloud, false); 
    mapToCloud(blue3d_hscaled, blue2d, markerImg, cloud, false); 

    EXPECT_EQ(red2d.size(), red3d_hscaled.size());
    EXPECT_EQ(green2d.size(), green3d_hscaled.size());
    EXPECT_EQ(yellow2d.size(), yellow3d_hscaled.size());
    EXPECT_EQ(blue2d.size(), blue3d_hscaled.size());
    */

    EXPECT_EQ(640, cloud.width);
    EXPECT_EQ(480, cloud.height);

    imshow("MapMarkers", markerImg);
    waitKey(-1);

    // TODO: PCLVisualizer segfaults for no reason...
    #ifdef ENABLE_PCLVIS
        // show the whole stuff
        PCLVisualizer vis;
        addPose(vis, PoseRT());

        addMarkerPolygon3d(vis, red3d, 255, 0, 0, "red");
        addMarkerPolygon3d(vis, green3d, 0, 216, 0, "green");
        addMarkerPolygon3d(vis, yellow3d, 255, 216, 0, "yellow");
        addMarkerPolygon3d(vis, blue3d, 0, 66, 255, "blue");
        /*
        addMarkerPolygon3d(vis, red3d_hscaled, 204, 153, 153, "red_hscaled");
        addMarkerPolygon3d(vis, green3d_hscaled, 153, 204, 153, "green_hscaled");
        addMarkerPolygon3d(vis, yellow3d_hscaled, 204, 204, 153, "yellow_hscaled");
        addMarkerPolygon3d(vis, blue3d_hscaled, 153, 153, 204, "blue_hscaled");*/

        vis.addPointCloud(cloud);
        vis.spin();
    #endif 
}

