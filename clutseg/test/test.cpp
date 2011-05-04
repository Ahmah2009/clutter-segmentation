/* 
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/common.h"

#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace cv;
using namespace opencv_candidate;
using namespace clutseg;

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
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



