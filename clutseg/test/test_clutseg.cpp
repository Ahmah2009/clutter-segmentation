/*
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

using namespace clutseg;
using namespace cv;
using namespace std;

TEST(ClutsegTest, DetectionWorks) {
   // Create segmenter
    ClutSegmenter segmenter(
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
        string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
    );

    Mat queryImage = imread("./data/image_00000.png");
    PointCloudT queryCloud;
    pcl::io::loadPCDFile("./data/cloud_00000.pcd", queryCloud);

    tod::Guess guess;
    PointCloudT inlierCloud;
    bool positive = segmenter.recognize(queryImage, queryCloud, guess, inlierCloud);
    EXPECT_TRUE(positive);
    EXPECT_EQ("haltbare_milch", guess.getObject()->name);
    cout << "detected: " << guess.getObject()->name << endl;
}

