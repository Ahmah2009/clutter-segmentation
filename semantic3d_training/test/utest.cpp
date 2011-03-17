/* 
 * Author: Julius Adorf
 */

#include <gtest/gtest.h>
#include <stdlib.h>
#include <cv.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include "misc.h"

using namespace std;

/** List all filenames in the current directory */
TEST(Misc, ListFiles)
{
    using namespace boost::filesystem;
    bool has_test = false;
    bool has_manifest = false;
    for (directory_iterator it("."), end; it != end; ++it) {
        string fname(it->path().filename());
        cout << fname << endl;
        if (fname.find("test") != string::npos) {
            has_test = true;
        } else if (fname.find("manifest.xml") != string::npos) {
            has_manifest = true;
        }
    }
    EXPECT_TRUE(has_test);
    EXPECT_TRUE(has_manifest);
}

/** List all filenames in the current directory */
TEST(Misc, ListFiles2)
{
    using namespace boost::filesystem;
    vector<path> v;
    copy(directory_iterator("."), directory_iterator(), back_inserter(v));
    bool has_test = false;
    bool has_manifest = false;
    for (vector<path>::const_iterator it(v.begin()); it != v.end(); ++it) {
        string fname = it->filename();
        cout << fname << endl;
        if (fname.find("test") != string::npos) {
            has_test = true;
        } else if (fname.find("manifest.xml") != string::npos) {
            has_manifest = true;
        }
    }
    EXPECT_TRUE(has_test);
    EXPECT_TRUE(has_manifest);
}

/** Extract an angle from a filename */
TEST(Misc, ExtractAngleFromFileName) {
    for (int a = -180; a <= 180; a += 30) {
        EXPECT_EQ(a, extractAngleFromFileName("bean-can_" + boost::lexical_cast<std::string>(a) + "_.log.delimited.rotated.pcd"));
    }
}

/** Read and write an image to an image file using OpenCV */
TEST(Misc, ReadWriteImage) {
    using namespace cv;
    Mat img(imread("./data/icetea2_0000_L.png.cropped.png"));
    imwrite("./data/icetea2_0000_L.png.cropped.out.png", img);
}

/** Display images using OpenCV */
TEST (Misc, DisplayImage) {
    using namespace cv;
    Mat img(imread("./data/icetea2_0000_L.png.cropped.png"));
    Mat img2(imread("./data/icetea2_0000_L.png.cropped.png", 0));
    namedWindow("image");
    imshow("image", img);
    namedWindow("gray-level image");
    imshow("gray-level image", img2);
    waitKey(5000);
}

TEST(Misc, ReadYaml) {
    using namespace cv;
    FileStorage fs("./data/config.yaml", FileStorage::READ);
    EXPECT_TRUE(fs.isOpened());
}

/** Read YAML file using OpenCV */
TEST(Misc, ExtractDetectorType) {
    using namespace cv;
    FileStorage fs("./data/config.yaml", FileStorage::READ);
    EXPECT_EQ("DynamicFAST", string(fs["TODParameters"]["feature_extraction_params"]["detector_type"]));
}

/** Read pose estimation from YAML file */
TEST(Misc, DeserializePoseFromYAML) {
    using namespace cv;
    using namespace opencv_candidate;

    FileStorage in("./data/pose.yaml", FileStorage::READ);
    PoseRT act_pose;
    act_pose.read(in[PoseRT::YAML_NODE_NAME]);
    EXPECT_EQ(2.0, act_pose.rvec.at<double>(0, 0));
    EXPECT_EQ(4.0, act_pose.rvec.at<double>(1, 0));
    EXPECT_EQ(8.0, act_pose.rvec.at<double>(2, 0));
    EXPECT_EQ(16.0, act_pose.tvec.at<double>(0, 0));
    EXPECT_EQ(32.0, act_pose.tvec.at<double>(1, 0));
    EXPECT_EQ(64.0, act_pose.tvec.at<double>(2, 0));
}

/** Serialize pose estimation to YAML file */
TEST(Misc, SerializePoseToYAML) {
    using namespace cv;
    using namespace opencv_candidate;

    PoseRT pose;
    pose.rvec = Mat::zeros(3, 1, 1); 
    pose.tvec = Mat::ones(3, 1, 1);

    FileStorage out("./data/pose.out.yaml", FileStorage::WRITE);
    out << PoseRT::YAML_NODE_NAME;
    pose.write(out);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

