/*
 * Author: Julius Adorf
 *
 * This is a test with which I want to find out which of the extractor
 * algorithms can be combined and which of them actually work as expected.  The
 * following aspects have to be evaluated to make them a possible factor in
 * parameter selection:
 * - whether keypoints are produced outside of the given mask
 * - how much time is required to extract features from an image
 * - whether the combination of (descriptor_type, extractor_type, detector_type)
 *   leads to the expected configuration
 * - how much influence do min_features and max_features have on the extraction
 *   process
 * - which parameter are influential for each combination of (descriptor_type,
 *   extractor_type, detector_type)
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
#include <boost/format.hpp>
#include <string>
#include <iostream>

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
                orb_harrisfast.detector_type = "ORB";
                orb_harrisfast.extractor_type = "ORB";
                orb_harrisfast.descriptor_type = "ORB";

                orb_harrisfast.extractor_params["octaves"] = 3;
                orb_harrisfast.detector_params["min_features"] = 300;
                orb_harrisfast.detector_params["max_features"] = 400;
            }

            {
                dynamicfast_multiscale_rbrief.detector_type = "DynamicFAST";
                dynamicfast_multiscale_rbrief.extractor_type = "multi-scale";
                dynamicfast_multiscale_rbrief.descriptor_type = "rBRIEF";

                dynamicfast_multiscale_rbrief.extractor_params["octaves"] = 3;
                dynamicfast_multiscale_rbrief.detector_params["min_features"] = 500;
                dynamicfast_multiscale_rbrief.detector_params["max_features"] = 700;
            }

            {
                sift_multiscale_rbrief.detector_type = "SIFT";
                sift_multiscale_rbrief.extractor_type = "multi-scale";
                sift_multiscale_rbrief.descriptor_type = "rBRIEF";

                sift_multiscale_rbrief.extractor_params["octaves"] = 3;
                sift_multiscale_rbrief.detector_params["min_features"] = 600;
                sift_multiscale_rbrief.detector_params["max_features"] = 900;
            }
 
            {
                sift_sequential_rbrief.detector_type = "SIFT";
                sift_sequential_rbrief.extractor_type = "sequential";
                sift_sequential_rbrief.descriptor_type = "rBRIEF";

                sift_sequential_rbrief.extractor_params["octaves"] = 3;
                sift_sequential_rbrief.detector_params["min_features"] = 500;
                sift_sequential_rbrief.detector_params["max_features"] = 700;
            } 
 
            {
                surf_sequential_rbrief.detector_type = "SURF";
                surf_sequential_rbrief.extractor_type = "sequential";
                surf_sequential_rbrief.descriptor_type = "rBRIEF";

                surf_sequential_rbrief.extractor_params["octaves"] = 3;
                surf_sequential_rbrief.detector_params["min_features"] = 600;
                surf_sequential_rbrief.detector_params["max_features"] = 900;
            }
 
            {
                dynamic_surf_sequential_rbrief.detector_type = "DynamicSURF";
                dynamic_surf_sequential_rbrief.extractor_type = "sequential";
                dynamic_surf_sequential_rbrief.descriptor_type = "rBRIEF";

                dynamic_surf_sequential_rbrief.extractor_params["octaves"] = 3;
                dynamic_surf_sequential_rbrief.detector_params["min_features"] = 600;
                dynamic_surf_sequential_rbrief.detector_params["max_features"] = 900;
            }
        }

        void expectMaskingWorks(const FeatureExtractionParams & params, bool expect = true) {
            Ptr<FeatureExtractor> extractor = FeatureExtractor::create(params);
            extractor->detectAndExtract(f2d_masked);
            EXPECT_EQ(CV_8U, f2d_masked.mask.type());
            // This proves that no keypoints have been detected outside of the region
            // of interest.
            vector<KeyPoint> outside;
            keypointsOutsideMask(f2d_masked.keypoints, f2d_masked.mask, outside);
            /* annoying-la Mat canvas;
            cvtColor(f2d_masked.mask, canvas, CV_GRAY2BGR);
            clutseg::drawKeypoints(canvas, f2d_masked.keypoints, Scalar::all(-1));
            clutseg::drawKeypoints(canvas, outside, Scalar(0, 0, 255));
            imshow("keypoints inside and outside of the mask", canvas);
            waitKey(1200); */
            if (expect) {
                EXPECT_EQ(0, outside.size());
            } else {
                EXPECT_LT(0, outside.size());
            }
        }

        void expectUsesMaxFeatures(FeatureExtractionParams params, int min_features, int max_features_high, int max_features_low) {
            params.detector_params["min_features"] = min_features;
            params.detector_params["max_features"] = max_features_high;
            FeatureExtractor::create(params, stats)->detectAndExtract(f2d);
            int more = f2d.keypoints.size();
            params.detector_params["max_features"] = max_features_low;
            f2d.keypoints = vector<KeyPoint>();
            FeatureExtractor::create(params, stats)->detectAndExtract(f2d);
            int less = f2d.keypoints.size();
            EXPECT_GT(more, less);
            EXPECT_TRUE(stats.pm_max_features_used);
        }

        void expectUsesMinFeatures(FeatureExtractionParams params, int max_features, int min_features_high, int min_features_low) {
            params.detector_params["max_features"] = max_features;
            params.detector_params["min_features"] = min_features_low;
            FeatureExtractor::create(params, stats)->detectAndExtract(f2d);
            int less = f2d.keypoints.size();
            params.detector_params["min_features"] = min_features_high;
            FeatureExtractor::create(params, stats)->detectAndExtract(f2d);
            int more = f2d.keypoints.size();
            EXPECT_LT(less, more);
            EXPECT_TRUE(stats.pm_min_features_used);
        }

        Features2d f2d;
        Features2d f2d_masked;
        detector_stats stats;
        
        // This configuration has been provided by ethanrublee.  It turns out
        // that max_features is taken into account but it is not an invariant,
        // just a guideline.
        FeatureExtractionParams orb_harrisfast;
        // This is the configuration  I have used in the first experiments with
        // tod_kinect and ias_kinect training data.
        FeatureExtractionParams dynamicfast_multiscale_rbrief;
        // This is kind of a weird configuration I have evaluated very quickly
        // without much thinking. It most probably does not make sense to wrap
        // SIFT with yet another multiscale extractor. Anyways, for sake of
        // understanding, see what happens... See ticket 1044 in OpenCV.
        // Requires post-processing.
        FeatureExtractionParams sift_multiscale_rbrief;
        // This is a new configuration (2011-05-04) I haven't tried before on
        // any training basee. It seems to be a viable option though. Keypoints
        // are detected outside of the mask, thus this requires post-processing
        // in tod_training. See ticket 1044 in OpenCV.
        FeatureExtractionParams sift_sequential_rbrief;
        // This is a new configuration (2011-05-04) I haven't tried it before
        // on any training base.
        FeatureExtractionParams surf_sequential_rbrief;
        // This is a new configuration (2011-05-04) I haven't tried it before
        // on any training base.
        FeatureExtractionParams dynamic_surf_sequential_rbrief;
};


// Check which combinations are definitely influenced by min_features 
// ---------------------------------------------------------------------------

TEST_F(ExtractorTest, orb_harrisfast_uses_max_features) {
    expectUsesMaxFeatures(orb_harrisfast, 300, 1000, 300);
}

TEST_F(ExtractorTest, dynamicfast_multiscale_rbrief_uses_max_features) {
    expectUsesMaxFeatures(dynamicfast_multiscale_rbrief, 0, 1000, 100);
}

TEST_F(ExtractorTest, dynamic_surf_sequential_rbrief_uses_max_features) {
    expectUsesMaxFeatures(dynamic_surf_sequential_rbrief, 0, 1000, 100);
}

// Check which combinations are definitely influenced by min_features. Whether
// a combination does not use the min_feature parameter cannot be decided.
// ---------------------------------------------------------------------------

TEST_F(ExtractorTest, dynamicfast_multiscale_rbrief_uses_min_features) {
    expectUsesMinFeatures(dynamicfast_multiscale_rbrief, 1200, 1000, 500);
}

TEST_F(ExtractorTest, dynamic_surf_sequential_rbrief_uses_min_features) {
    expectUsesMinFeatures(dynamic_surf_sequential_rbrief, 1200, 1000, 500);
}


// Check whether the mask parameter is correctly used
// ---------------------------------------------------------------------------

TEST_F(ExtractorTest, orb_harrisfast_masking_works) {
    expectMaskingWorks(orb_harrisfast);
}

TEST_F(ExtractorTest, dynamicfast_multiscale_rbrief_masking_works) {
    expectMaskingWorks(dynamicfast_multiscale_rbrief);
}

// See OpenCV ticket 1044
#ifdef OPENCV_R5024 
    TEST_F(ExtractorTest, sift_multiscale_rbrief_masking_works) {
        expectMaskingWorks(sift_multiscale_rbrief);
    }
#else
    TEST_F(ExtractorTest, sift_multiscale_rbrief_masking_fails) {
        expectMaskingWorks(sift_multiscale_rbrief, false);
    }
#endif

#ifdef OPENCV_R5024 
    TEST_F(ExtractorTest, sift_sequential_rbrief_masking_works) {
        expectMaskingWorks(sift_sequential_rbrief);
    }
#else
    TEST_F(ExtractorTest, sift_sequential_rbrief_masking_fails) {
        // expect to fail because of OpenCV 1044
        expectMaskingWorks(sift_sequential_rbrief, false);
    }
#endif

TEST_F(ExtractorTest, surf_sequential_rbrief_masking_works) {
    expectMaskingWorks(surf_sequential_rbrief);
}

TEST_F(ExtractorTest, dynamic_surf_sequential_rbrief_masking_works) {
    expectMaskingWorks(dynamic_surf_sequential_rbrief);
}

// Check whether expectations on the resulting configuration are met
// ---------------------------------------------------------------------------

TEST_F(ExtractorTest, orb_harrisfast_config) {
    FeatureExtractor::create(orb_harrisfast, stats);
    EXPECT_EQ("<none>", stats.internal_detector);
    EXPECT_EQ("<none>", stats.internal_extractor);
    EXPECT_EQ("tod::OrbExtractor with HarrisFast", stats.extractor);
}

TEST_F(ExtractorTest, dynamicfast_multiscale_rbrief_config) {
    FeatureExtractor::create(dynamicfast_multiscale_rbrief, stats);
    cout << stats.internal_detector << endl;
    EXPECT_EQ("cv::DynamicAdaptedFeatureDetector with tod::RBriefFastAdjuster", stats.internal_detector);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::MultiscaleExtractor", stats.extractor);
}
 
TEST_F(ExtractorTest, sift_multiscale_rbrief_config) {
    FeatureExtractor::create(sift_multiscale_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SiftFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::MultiscaleExtractor", stats.extractor);
}

TEST_F(ExtractorTest, sift_sequential_rbrief_config) {
    FeatureExtractor::create(sift_sequential_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SiftFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

TEST_F(ExtractorTest, surf_sequential_rbrief_config) {
    FeatureExtractor::create(surf_sequential_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SurfFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

TEST_F(ExtractorTest, dynamic_surf_sequential_rbrief_config) {
    FeatureExtractor::create(dynamic_surf_sequential_rbrief, stats);
    EXPECT_EQ("cv::DynamicAdaptedFeatureDetector with cv::SurfAdjuster", stats.internal_detector);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}


// Miscellaneous other tests
// ---------------------------------------------------------------------------

#ifndef OPENCV_R5024 
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
#endif

// Finding [min_features, max_features] windows that are worth trying for
// feature extractors
// ---------------------------------------------------------------------------

/*
struct min_max_features_win {

    min_max_features_win(int midpoint, int width) : midpoint(midpoint), width(width) {}

    int midpoint;
    int width;

}; comparable?!

TEST_F(ExtractorTest, dynamic_surf_sequential_rbrief_windows) {
    // Take a single image and a mask and detect features using different [min_features, max_features]
    // windows. Collect results in a map.
    map<min_max_features_win, int> res;
    for (int w = 0; w < 500; w += 50) {
        for (int m = 0; m < 1500; m += 50) {
            f2d_masked.keypoints.clear();
            min_max_features_win win(m, w);
            dynamic_surf_sequential_rbrief.detector_params["min_features"] = win.midpoint - win.width / 2;
            dynamic_surf_sequential_rbrief.detector_params["max_features"] = win.midpoint + win.width / 2;
            FeatureExtractor::create(dynamic_surf_sequential_rbrief)->detectAndExtract(f2d_masked);
            res[win] = (int) f2d_masked.keypoints.size(); 
        }
    }

    map<min_max_features_win, int>::iterator it = res.begin();
    map<min_max_features_win, int>::iterator end;
    while (it != end) {
        cout << boost::format("midpoint: %4d, width: %4d, keypoints: %4d") % (it->first).midpoint % (it->first).width % it->second << endl;
        it++;
    }
}*/
