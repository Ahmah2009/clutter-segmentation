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
#include <boost/lexical_cast.hpp>
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

class test_extractor : public ::testing::Test {
    public:
        virtual void SetUp() {
            f2d.image = imread("./data/image_00000.png", 0);
            f2d_masked.image = f2d.image.clone(); 
            f2d_masked.mask = imread("./data/image_00000.png.mask.png", 0);

            string p(getenv("CLUTSEG_PATH"));

            // TODO: these train images are not included in repository
            train_images.push_back(imread(p + "/ias_kinect_train_v2/assam_tea/image_00000.png", 0));
            train_images.push_back(imread(p + "/ias_kinect_train_v2/haltbare_milch/image_00000.png", 0));
            train_images.push_back(imread(p + "/ias_kinect_train_v2/icedtea/image_00000.png", 0));
            train_images.push_back(imread(p + "/ias_kinect_train_v2/jacobs_coffee/image_00000.png", 0));

            // TODO: these test images are not included in repository
            test_images.push_back(imread(p + "/ias_kinect_test_all/all01/image_00000.jpg", 0));
            test_images.push_back(imread(p + "/ias_kinect_test_all/all02/image_00000.jpg", 0));
            test_images.push_back(imread(p + "/ias_kinect_test_all/all03/image_00000.jpg", 0));
            test_images.push_back(imread(p + "/ias_kinect_test_grounded_21/at_hm_jc/image_00008.png", 0));

            {
                feparams["orb_harrisfast"] = &orb_harrisfast;
                orb_harrisfast.detector_type = "ORB-TOD";
                orb_harrisfast.extractor_type = "ORB-TOD";
                orb_harrisfast.descriptor_type = "ORB-TOD";

                orb_harrisfast.extractor_params["octaves"] = 3;
                orb_harrisfast.detector_params["min_features"] = 300;
                orb_harrisfast.detector_params["max_features"] = 400;
            }

            {
                feparams["orb_opencv"] = &orb_opencv;
                orb_opencv.detector_type = "ORB";
                orb_opencv.extractor_type = "ORB";
                orb_opencv.descriptor_type = "ORB";

                orb_opencv.extractor_params["octaves"] = 3;
                orb_opencv.extractor_params["scale_factor"] = 1.2f;
                orb_opencv.detector_params["n_features"] = 500;
            }

            {
                feparams["orb_opencv_custom"] = &orb_opencv_custom;
                orb_opencv_custom.detector_type = "ORB";
                orb_opencv_custom.extractor_type = "ORB";
                orb_opencv_custom.descriptor_type = "ORB";

                orb_opencv_custom.extractor_params["octaves"] = 2;
                orb_opencv_custom.extractor_params["scale_factor"] = 1.5f;
                orb_opencv_custom.detector_params["n_features"] = 5000;
            }

            {
                feparams["fast_multiscale_rbrief"] = &fast_multiscale_rbrief;
                fast_multiscale_rbrief.detector_type = "FAST";
                fast_multiscale_rbrief.extractor_type = "multi-scale";
                fast_multiscale_rbrief.descriptor_type = "rBRIEF";

                fast_multiscale_rbrief.extractor_params["octaves"] = 3;
            }

            {
                feparams["dynamicfast_multiscale_rbrief"] = &dynamicfast_multiscale_rbrief;
                dynamicfast_multiscale_rbrief.detector_type = "DynamicFAST";
                dynamicfast_multiscale_rbrief.extractor_type = "multi-scale";
                dynamicfast_multiscale_rbrief.descriptor_type = "rBRIEF";

                dynamicfast_multiscale_rbrief.extractor_params["octaves"] = 3;
                dynamicfast_multiscale_rbrief.detector_params["min_features"] = 500;
                dynamicfast_multiscale_rbrief.detector_params["max_features"] = 700;
            }

            {
                feparams["sift_sequential_rbrief"] = &sift_sequential_rbrief;
                sift_sequential_rbrief.detector_type = "SIFT";
                sift_sequential_rbrief.extractor_type = "sequential";
                sift_sequential_rbrief.descriptor_type = "rBRIEF";

                sift_sequential_rbrief.extractor_params["octaves"] = 3;
                sift_sequential_rbrief.detector_params["min_features"] = 500;
                sift_sequential_rbrief.detector_params["max_features"] = 700;
            } 
 
            {
                feparams["surf_sequential_rbrief"] = &surf_sequential_rbrief;
                surf_sequential_rbrief.detector_type = "SURF";
                surf_sequential_rbrief.extractor_type = "sequential";
                surf_sequential_rbrief.descriptor_type = "rBRIEF";

                surf_sequential_rbrief.extractor_params["octaves"] = 3;
                surf_sequential_rbrief.detector_params["min_features"] = 600;
                surf_sequential_rbrief.detector_params["max_features"] = 900;
            }
 
            {
                feparams["dynamicsurf_sequential_rbrief"] = &dynamicsurf_sequential_rbrief;
                dynamicsurf_sequential_rbrief.detector_type = "DynamicSURF";
                dynamicsurf_sequential_rbrief.extractor_type = "sequential";
                dynamicsurf_sequential_rbrief.descriptor_type = "rBRIEF";

                dynamicsurf_sequential_rbrief.extractor_params["octaves"] = 3;
                dynamicsurf_sequential_rbrief.detector_params["min_features"] = 600;
                dynamicsurf_sequential_rbrief.detector_params["max_features"] = 900;
            }

            {
                feparams["star_sequential_rbrief"] = &star_sequential_rbrief;
                star_sequential_rbrief.detector_type = "STAR";
                star_sequential_rbrief.extractor_type = "sequential";
                star_sequential_rbrief.descriptor_type = "rBRIEF";

                star_sequential_rbrief.extractor_params["octaves"] = 3;
            }

            {
                feparams["dynamicstar_sequential_rbrief"] = &dynamicstar_sequential_rbrief;
                dynamicstar_sequential_rbrief.detector_type = "DynamicSTAR";
                dynamicstar_sequential_rbrief.extractor_type = "sequential";
                dynamicstar_sequential_rbrief.descriptor_type = "rBRIEF";

                dynamicstar_sequential_rbrief.extractor_params["octaves"] = 3;
            }

            {
                feparams["mser_multiscale_rbrief"] = &mser_multiscale_rbrief;
                mser_multiscale_rbrief.detector_type = "MSER";
                mser_multiscale_rbrief.extractor_type = "multi-scale";
                mser_multiscale_rbrief.descriptor_type = "rBRIEF";

                mser_multiscale_rbrief.extractor_params["octaves"] = 3;
            }

            {
                feparams["gftt_multiscale_rbrief"] = &gftt_multiscale_rbrief;
                gftt_multiscale_rbrief.detector_type = "GFTT";
                gftt_multiscale_rbrief.extractor_type = "multi-scale";
                gftt_multiscale_rbrief.descriptor_type = "rBRIEF";

                gftt_multiscale_rbrief.extractor_params["octaves"] = 3;
            }
        }

        bool expectMaskingWorks(const FeatureExtractionParams & params, bool expect = true) {
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
                return outside.size() == 0;
            } else {
                EXPECT_LT(0, outside.size());
                return outside.size() > 0;
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

        int expectRunsWithin(const FeatureExtractionParams & params, const vector<Mat> & images, int ms) {
            Ptr<FeatureExtractor> e = FeatureExtractor::create(params);
            clock_t before = clock();
            Features2d f;
            for (size_t i = 0; i < images.size(); i++) {
                f.image = images[i];
                e->detectAndExtract(f);
            }
            clock_t after = clock();
            int avg = 1000 * (after - before) / CLOCKS_PER_SEC / images.size();
            EXPECT_LT(avg, ms);
            return avg;
        }

        Features2d f2d;
        Features2d f2d_masked;
        detector_stats stats;
        
        // This configuration has been provided by ethanrublee.  It turns out
        // that max_features is taken into account but it is not an invariant,
        // just a guideline.
        FeatureExtractionParams orb_harrisfast;
        // ORB configuration for OpenCV ORB, uses mainly default parameters
        FeatureExtractionParams orb_opencv;
        // ORB configuration for OpenCV ORB, uses customized parameters
        FeatureExtractionParams orb_opencv_custom;
        FeatureExtractionParams fast_multiscale_rbrief;
        // This is the configuration  I have used in the first experiments with
        // tod_kinect and ias_kinect training data.
        FeatureExtractionParams dynamicfast_multiscale_rbrief;
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
        FeatureExtractionParams dynamicsurf_sequential_rbrief;
        FeatureExtractionParams star_sequential_rbrief;
        FeatureExtractionParams dynamicstar_sequential_rbrief;
        FeatureExtractionParams mser_multiscale_rbrief;
        FeatureExtractionParams gftt_multiscale_rbrief;
    
        map<const string, FeatureExtractionParams*> feparams;

        vector<Mat> train_images;
        vector<Mat> test_images;
};

// Validate test data
// ---------------------------------------------------------------------------
TEST_F(test_extractor, validate_train_images) {
    BOOST_FOREACH(const Mat & img, train_images) {
        EXPECT_FALSE(img.empty());
    }
}

TEST_F(test_extractor, validate_test_images) {
    BOOST_FOREACH(const Mat & img, test_images) {
        EXPECT_FALSE(img.empty());
    }
}

// Check which combinations are definitely influenced by min_features 
// ---------------------------------------------------------------------------

TEST_F(test_extractor, orb_harrisfast_uses_max_features) {
    expectUsesMaxFeatures(orb_harrisfast, 300, 1000, 300);
}

TEST_F(test_extractor, dynamicfast_multiscale_rbrief_uses_max_features) {
    expectUsesMaxFeatures(dynamicfast_multiscale_rbrief, 0, 1000, 100);
}

TEST_F(test_extractor, dynamicsurf_sequential_rbrief_uses_max_features) {
    SKIP_IF_FAST

    expectUsesMaxFeatures(dynamicsurf_sequential_rbrief, 0, 1000, 100);
}

TEST_F(test_extractor, dynamicstar_sequential_rbrief_uses_max_features) {
    expectUsesMaxFeatures(dynamicstar_sequential_rbrief, 0, 1000, 100);
}

// Check which combinations are definitely influenced by min_features. Whether
// a combination does not use the min_feature parameter cannot be decided.
// ---------------------------------------------------------------------------

TEST_F(test_extractor, dynamicfast_multiscale_rbrief_uses_min_features) {
    expectUsesMinFeatures(dynamicfast_multiscale_rbrief, 1200, 1000, 500);
}

TEST_F(test_extractor, dynamicsurf_sequential_rbrief_uses_min_features) {
    SKIP_IF_FAST

    expectUsesMinFeatures(dynamicsurf_sequential_rbrief, 1200, 1000, 500);
}


// Check whether the mask parameter is correctly used
// ---------------------------------------------------------------------------

TEST_F(test_extractor, masking_works) {
    SKIP_IF_FAST

    map<const string, FeatureExtractionParams*>::iterator it = feparams.begin();
    map<const string, FeatureExtractionParams*>::iterator end = feparams.end();
    while (it != end) {
        bool ok = expectMaskingWorks(*(it->second));
        cout << boost::format("%35s [%4s]") % it->first % (ok ? "OK" : "FAIL") << endl;
        it++;
    }
}

// Check whether expectations on the resulting configuration are met
// ---------------------------------------------------------------------------

TEST_F(test_extractor, orb_harrisfast_config) {
    FeatureExtractor::create(orb_harrisfast, stats);
    EXPECT_EQ("<none>", stats.internal_detector);
    EXPECT_EQ("<none>", stats.internal_extractor);
    EXPECT_EQ("tod::OrbExtractor with HarrisFast", stats.extractor);
}

TEST_F(test_extractor, orb_opencv_config) {
    FeatureExtractor::create(orb_opencv, stats);
    EXPECT_EQ("<none>", stats.internal_detector);
    EXPECT_EQ("<none>", stats.internal_extractor);
    EXPECT_EQ("cv::ORB", stats.extractor);
}

TEST_F(test_extractor, orb_opencv_custom_config) {
    FeatureExtractor::create(orb_opencv_custom, stats);
    EXPECT_EQ("<none>", stats.internal_detector);
    EXPECT_EQ("<none>", stats.internal_extractor);
    EXPECT_EQ("cv::ORB", stats.extractor);
}

TEST_F(test_extractor, dynamicfast_multiscale_rbrief_config) {
    FeatureExtractor::create(dynamicfast_multiscale_rbrief, stats);
    cout << stats.internal_detector << endl;
    EXPECT_EQ("cv::DynamicAdaptedFeatureDetector with tod::RBriefFastAdjuster", stats.internal_detector);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::MultiscaleExtractor", stats.extractor);
}
 
TEST_F(test_extractor, sift_sequential_rbrief_config) {
    FeatureExtractor::create(sift_sequential_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SiftFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

TEST_F(test_extractor, surf_sequential_rbrief_config) {
    FeatureExtractor::create(surf_sequential_rbrief, stats);
    EXPECT_TRUE(stats.internal_detector.find("SurfFeatureDetector") != string::npos);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

TEST_F(test_extractor, dynamicsurf_sequential_rbrief_config) {
    FeatureExtractor::create(dynamicsurf_sequential_rbrief, stats);
    EXPECT_EQ("cv::DynamicAdaptedFeatureDetector with cv::SurfAdjuster", stats.internal_detector);
    EXPECT_EQ("rbrief::RBriefDescriptorExtractor", stats.internal_extractor);
    EXPECT_EQ("tod::SequentialExtractor", stats.extractor);
}

// Check whether certain configurations run within time bounds, and also write
// down statistics.
// ---------------------------------------------------------------------------

TEST_F(test_extractor, times) {
    SKIP_IF_FAST

    map<const string, FeatureExtractionParams*>::iterator it = feparams.begin();
    map<const string, FeatureExtractionParams*>::iterator end = feparams.end();
    ofstream out("build/test_extractor.times");
    out << boost::format("%35s %5s %5s") % "feparams" % "ttrain" % "ttest" << endl;
    while (it != end) {
        int t1 = expectRunsWithin(*(it->second), train_images, 5000);
        int t2 = expectRunsWithin(*(it->second), test_images, 5000);
        out << boost::format("%35s %5d %5d") % it->first % t1 % t2 << endl;
        it++;
    }
    out.close();
}

// Miscellaneous other tests
// ---------------------------------------------------------------------------

#ifndef OPENCV_R5024 
    TEST_F(test_extractor, sift_feature_detector_ignoresMask) {
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

struct min_max_features_win {

    min_max_features_win(int midpoint, int width) : midpoint(midpoint), width(width) {}

    int midpoint;
    int width;

    bool operator<(const min_max_features_win & w) const {
        return (midpoint < w.midpoint ? true :
                  (midpoint > w.midpoint ? false : width < w.width));
    }

};

TEST_F(test_extractor, dynamicsurf_sequential_rbrief_windows) {
    SKIP_IF_FAST

    // Take a single image and a mask and detect features using different [min_features, max_features]
    // windows. Collect results in a map.
    map<min_max_features_win, int> res;
    for (int w = 100; w <= 500; w += 100) {
        for (int m = 0; m <= 1000; m += 500) {
            f2d_masked.keypoints.clear();
            min_max_features_win win(m, w);
            dynamicsurf_sequential_rbrief.detector_params["min_features"] = win.midpoint - win.width / 2;
            dynamicsurf_sequential_rbrief.detector_params["max_features"] = win.midpoint + win.width / 2;
            FeatureExtractor::create(dynamicsurf_sequential_rbrief)->detectAndExtract(f2d_masked);
            res[win] = (int) f2d_masked.keypoints.size(); 
            cout << boost::format("midpoint: %4d, width: %4d, keypoints: %4d") % win.midpoint % win.width % f2d_masked.keypoints.size() << endl;
        }
    }

    map<min_max_features_win, int>::iterator it = res.begin();
    map<min_max_features_win, int>::iterator end = res.end();
    while (it != end) {
        cout << boost::format("midpoint: %4d, width: %4d, keypoints: %4d") % (it->first).midpoint % (it->first).width % it->second << endl;
        it++;
    }
}

// Testing ORB from OpenCV trunk
// ---------------------------------------------------------------------------

TEST_F(test_extractor, orb_extract_features) {
    cv::ORB orb = cv::ORB(5000);
    orb(f2d.image, f2d.mask, f2d.keypoints);

    if (!fast()) {
        Mat c;
        drawKeypoints(f2d.image, f2d.keypoints, c);
        imshow("orb_extract_features", c);
        waitKey(-1);
    }
}

TEST_F(test_extractor, orb_opencv_extract_features) {
    Ptr<FeatureExtractor> orb = FeatureExtractor::create(orb_opencv, stats);
    orb->detectAndExtract(f2d);

    if (!fast()) {
        Mat c;
        drawKeypoints(f2d.image, f2d.keypoints, c);
        imshow("orb_opencv_extract_features", c);
        waitKey(-1);
    }
}

TEST_F(test_extractor, orb_opencv_custom_extract_features) {
    Ptr<FeatureExtractor> orb = FeatureExtractor::create(orb_opencv_custom, stats);
    orb->detectAndExtract(f2d);

    if (!fast()) {
        Mat c;
        drawKeypoints(f2d.image, f2d.keypoints, c);
        imshow("orb_opencv_custom_extract_features", c);
        waitKey(-1);
    }
}

