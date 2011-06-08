/* 
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/common.h"
#include "clutseg/flags.h"

#include <gtest/gtest.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace std;

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

bool fast() {
    FileFlag f("build/fast.flag");
    return f.exists();
}

void fast_warning() {
    cerr << "[WARNING]: Test skipped." << endl;
}

void sampleColorImage(Mat & img) {
    img = imread("./data/image_00000.png");
}

void samplePose(PoseRT & pose) {
    FileStorage in("./data/image_00000.png.pose.yaml", FileStorage::READ);
    pose.read(in[PoseRT::YAML_NODE_NAME]);
}

void sampleCloud(PointCloudT & cloud) {
    pcl::io::loadPCDFile("./data/cloud_00000.pcd", cloud);
}

