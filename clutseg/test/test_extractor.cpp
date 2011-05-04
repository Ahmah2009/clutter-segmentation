/*
 * Author: Julius Adorf
 *
 * This is a test with which I want to find out which of the extractor
 * algorithms can be combined and which of them actually work as expected.
 */

#include "test.h"

#include <gtest/gtest.h>

#include "clutseg/viz.h"

#include <stdint.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <tod/core/Features2d.h>
#include <tod/training/feature_extraction.h>
#include <boost/foreach.hpp>
#include <string>

using namespace std;
using namespace tod;
using namespace cv;

void keypointsOutsideMask(const vector<KeyPoint> keypoints, const Mat & mask, vector<KeyPoint> & outside) {
     // This proves that no keypoint has been detected outside of the region of
    // interest. Due to some boundary phenomena, we also check the neighbors. If
    // none of the neighbors in a 3x3 patch is included, then we declare the
    // extractor as "ignoring the mask".
    BOOST_FOREACH(const KeyPoint & p, keypoints) {
        Rect n(p.pt.x - 2, p.pt.y - 2, 5, 5);
        bool w = false;
        for(int y = n.y; y < n.y + n.height; y++) {
            for(int x = n.x; x < n.x + n.width; x++) {
                if (mask.at<uint8_t>(y, x) > 0) {
                    w = true;
                }
            }
        }
        if (!w) {
            outside.push_back(p);
        }
    }
}

class ExtractorTest : public ::testing::Test {
    public:
        virtual void SetUp() {
            f2d.image = imread("./data/image_00000.png", 0);
            f2d_masked.image = f2d.image.clone(); 
            f2d_masked.mask = imread("./data/image_00000.png.mask.png", 0);

            {
                // This configuration has been provided by ethanrublee.
                // It turns out that max_features is taken into account but
                // it is not an invariant, just a guideline.
                orb_harrisfast.detector_type = "ORB";
                orb_harrisfast.extractor_type = "ORB";
                orb_harrisfast.descriptor_type = "ORB";

                orb_harrisfast.extractor_params["octaves"] = 3;
                orb_harrisfast.detector_params["min_features"] = 300;
                orb_harrisfast.detector_params["max_features"] = 400;
            }

            {
                // This is the configuration  I have used in the first
                // experiments with tod_kinect and ias_kinect training data.
                dynamicfast_multiscale_rbrief.detector_type = "DynamicFAST";
                dynamicfast_multiscale_rbrief.extractor_type = "multi-scale";
                dynamicfast_multiscale_rbrief.descriptor_type = "rBRIEF";

                dynamicfast_multiscale_rbrief.extractor_params["octaves"] = 3;
                dynamicfast_multiscale_rbrief.detector_params["min_features"] = 500;
                dynamicfast_multiscale_rbrief.detector_params["max_features"] = 700;
            }

            {
                // This is kind of a weird configuration I have evaluated very
                // quickly without much thinking. It most probably does not
                // make sense to wrap SIFT with yet another multiscale
                // extractor. Anyways, for sake of understanding, see what 
                // happens...
                sift_multiscale_rbrief.detector_type = "SIFT";
                sift_multiscale_rbrief.extractor_type = "multi-scale";
                sift_multiscale_rbrief.descriptor_type = "rBRIEF";

                sift_multiscale_rbrief.extractor_params["octaves"] = 3;
                sift_multiscale_rbrief.detector_params["min_features"] = 600;
                sift_multiscale_rbrief.detector_params["max_features"] = 900;
            }
 
            {
                // This is a new configuration (2011-05-04) I haven't tried
                // before on any training sense. It seems to make sense to me
                // though and there is the question where it really detects
                // keypoints only within the mask.
                sift_sequential_rbrief.detector_type = "SIFT";
                sift_sequential_rbrief.extractor_type = "sequential";
                sift_sequential_rbrief.descriptor_type = "rBRIEF";

                sift_sequential_rbrief.extractor_params["octaves"] = 3;
                sift_sequential_rbrief.detector_params["min_features"] = 500;
                sift_sequential_rbrief.detector_params["max_features"] = 700;
            } 
        }

        void expectMaskingWorks(const FeatureExtractionParams & params) {
            Ptr<FeatureExtractor> extractor = FeatureExtractor::create(params);
            extractor->detectAndExtract(f2d_masked);
            EXPECT_EQ(CV_8U, f2d_masked.mask.type());
            // This proves that no keypoints have been detected outside of the region
            // of interest.
            vector<KeyPoint> outside;
            keypointsOutsideMask(f2d_masked.keypoints, f2d_masked.mask, outside);
            Mat canvas;
            cvtColor(f2d_masked.mask, canvas, CV_GRAY2BGR);
            clutseg::drawKeypoints(canvas, f2d_masked.keypoints, Scalar::all(-1));
            clutseg::drawKeypoints(canvas, outside, Scalar(0, 0, 255));
            imshow("keypoints inside and outside of the mask", canvas);
            waitKey(-1);
            EXPECT_EQ(0, outside.size());
        }

        Features2d f2d;
        Features2d f2d_masked;
        detector_stats stats;
        FeatureExtractionParams orb_harrisfast;
        FeatureExtractionParams dynamicfast_multiscale_rbrief;
        FeatureExtractionParams sift_multiscale_rbrief;
        FeatureExtractionParams sift_sequential_rbrief;
};


TEST_F(ExtractorTest, OrbExtractorHarrisFastStats) {
    detector_stats stats;

    Ptr<FeatureExtractor> extractor = FeatureExtractor::create(orb_harrisfast, stats);
    extractor->detectAndExtract(f2d);

    EXPECT_EQ("<none>", stats.internal_detector);
    EXPECT_EQ("<none>", stats.internal_extractor);
    EXPECT_EQ("tod::OrbExtractor with HarrisFast", stats.extractor);
}

TEST_F(ExtractorTest, OrbExtractorHarrisFastNumberOfKpts) {
    Ptr<FeatureExtractor> extractor = FeatureExtractor::create(orb_harrisfast, stats);
    extractor->detectAndExtract(f2d);
    // The guideline keypoints.size() > min_features will be met by the
    // extractor on the given example image.
    EXPECT_GE(f2d.keypoints.size(), orb_harrisfast.detector_params["min_features"]);
    // It does not meet the guideline for the maximum number of features though.
    EXPECT_GE(f2d.keypoints.size(), orb_harrisfast.detector_params["max_features"]);
    EXPECT_EQ(1267, f2d.keypoints.size());
}

TEST_F(ExtractorTest, OrbExtractorHarrisFastZero) {
    orb_harrisfast.detector_params["max_features"] = 0;
    Ptr<FeatureExtractor> extractor = FeatureExtractor::create(orb_harrisfast, stats);
    extractor->detectAndExtract(f2d);
    // This just proves that the max_features parameter is actually taken into
    // account here.
    EXPECT_EQ(f2d.keypoints.size(), 0);
}

TEST_F(ExtractorTest, orb_harrisfast_masking_works) {
    expectMaskingWorks(orb_harrisfast);
}

TEST_F(ExtractorTest, dynamicfast_multiscale_rbrief_masking_works) {
    // This is a bug in OpenCV. The masking parameter is useless.
    expectMaskingWorks(dynamicfast_multiscale_rbrief);
}

TEST_F(ExtractorTest, sift_multiscale_rbrief_stats) {
    Ptr<FeatureExtractor> extractor = FeatureExtractor::create(sift_multiscale_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SiftFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::MultiscaleExtractor", stats.extractor);
}

TEST_F(ExtractorTest, sift_multiscale_rbrief_masking_works) {
    // This is a bug in OpenCV. The masking parameter is useless.
    expectMaskingWorks(sift_multiscale_rbrief);
}

TEST_F(ExtractorTest, sift_sequential_rbrief_stats) {
    Ptr<FeatureExtractor> extractor = FeatureExtractor::create(sift_sequential_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SiftFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

TEST_F(ExtractorTest, sift_sequential_rbrief_masking_works) {
    expectMaskingWorks(sift_sequential_rbrief);
}

TEST_F(ExtractorTest, SiftFeatureDetectorIgnoresMask) {
    // I expect having found an annoying bug in OpenCV
    // SiftFeatureDetector does not use the mask.
    // see http://opencv-users.1802565.n2.nabble.com/Problems-about-the-OpenCV-SIFT-feature-detector-td6084481.html
    // see https://code.ros.org/trac/opencv/ticket/1029
    SiftFeatureDetector fd = SiftFeatureDetector(SIFT::DetectorParams::
                                         GET_DEFAULT_THRESHOLD(),
                                         SIFT::DetectorParams::
                                         GET_DEFAULT_EDGE_THRESHOLD());
    fd.detect(f2d_masked.image, f2d_masked.keypoints, f2d_masked.mask);
    vector<KeyPoint> outside;
    keypointsOutsideMask(f2d_masked.keypoints, f2d_masked.mask, outside);
    // If this test passes, then OpenCV SiftFeatureDetector::detect is buggy
    // and ignores the mask.
    EXPECT_GT(outside.size(), 0);
}

