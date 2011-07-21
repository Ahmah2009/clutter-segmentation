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

#include "clutseg/ground.h"
#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include <algorithm>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <map>
#include <stdint.h>
#include <string>
#include <tod/core/Features2d.h>
#include <tod/training/feature_extraction.h>
#include <utility>
#include <vector>

using namespace clutseg;
using namespace std;
using namespace tod;
using namespace cv;

namespace bfs = boost::filesystem;

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
                feparams["orb_opencv"] = &orb_opencv;
                orb_opencv.detector_type = "ORB";
                orb_opencv.extractor_type = "ORB";
                orb_opencv.descriptor_type = "ORB";

                orb_opencv.extractor_params["octaves"] = 3;
                orb_opencv.extractor_params["scale_factor"] = 1.2f;
                orb_opencv.extractor_params["n_features"] = 500;
                orb_opencv.detector_params["octaves"] = 3;
                orb_opencv.detector_params["scale_factor"] = 1.2f;
                orb_opencv.detector_params["n_features"] = 500;
            }

            {
                feparams["orb_opencv_custom"] = &orb_opencv_custom;
                orb_opencv_custom.detector_type = "ORB";
                orb_opencv_custom.extractor_type = "ORB";
                orb_opencv_custom.descriptor_type = "ORB";

                orb_opencv_custom.extractor_params["octaves"] = 2;
                orb_opencv_custom.extractor_params["scale_factor"] = 1.5f;
                orb_opencv_custom.extractor_params["n_features"] = 5000;
                orb_opencv_custom.detector_params["octaves"] = 2;
                orb_opencv_custom.detector_params["scale_factor"] = 1.5f;
                orb_opencv_custom.detector_params["n_features"] = 5000;
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
         
            Mat canvas;
            cvtColor(f2d_masked.mask, canvas, CV_GRAY2BGR);
            clutseg::drawKeypoints(canvas, f2d_masked.keypoints, Scalar::all(-1));
            clutseg::drawKeypoints(canvas, outside, Scalar(0, 0, 255));
            imshow(params.detector_type, canvas);
            waitKey(-1);
   
            if (expect) {
                EXPECT_EQ(0, outside.size());
                return outside.size() == 0;
            } else {
                EXPECT_LT(0, outside.size());
                return outside.size() > 0;
            }
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
        
        // ORB configuration for OpenCV ORB, uses mainly default parameters
        FeatureExtractionParams orb_opencv;
        // ORB configuration for OpenCV ORB, uses customized parameters
        FeatureExtractionParams orb_opencv_custom;
    
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

#if ! defined OPENCV_R5024 && ! defined OPENCV_R5465
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
    Ptr<FeatureExtractor> orb = FeatureExtractor::create(orb_opencv);
    orb->detectAndExtract(f2d);

    if (!fast()) {
        Mat c;
        drawKeypoints(f2d.image, f2d.keypoints, c);
        imshow("orb_opencv_extract_features", c);
        waitKey(-1);
    }
}

TEST_F(test_extractor, orb_opencv_custom_extract_features) {
    Ptr<FeatureExtractor> orb = FeatureExtractor::create(orb_opencv_custom);
    orb->detectAndExtract(f2d);

    if (!fast()) {
        Mat c;
        drawKeypoints(f2d.image, f2d.keypoints, c);
        imshow("orb_opencv_custom_extract_features", c);
        waitKey(-1);
    }
}

// SIFT-SURF-ORB Benchmark 
// ---------------------------------------------------------------------------

struct benchmark_t {
    benchmark_t() : time_cpu(0), time_real(0), quantity(0), n(0) {}
    Mat sample_img;
    double time_cpu;
    double time_real;
    int quantity;
    int n;
    static void header() {
        cout << boost::format("%9s %8s %8s %8s %8s %8s")
            % "extractor"
            % "img_cpu"
            % "img_real"
            % "kpt_cpu"
            % "kpt_real"
            % "quantity"
                << endl;
    }
    void print(const string & name) const {
        cout << boost::format("%9s %8.2f %8.2f %8.2f %8.2f %8.0f")
            % name
            % (time_cpu / n)
            % (time_real / n)
            % (time_cpu / quantity)
            % (time_real / quantity)
            % (quantity / n)
                << endl;
    }
};

template<typename F>
benchmark_t benchmark(F extract, const vector<bfs::path> & image_paths) {
    benchmark_t bm;
    int i = 0;
    BOOST_FOREACH(const bfs::path & p, image_paths) {
        cout << "." << flush;
        Mat img = imread(p.string(), 0);
        Mat mask = Mat::ones(img.size(), CV_8UC1);
        vector<KeyPoint> kpts;
        // timing start
        timeval at;
        assert(gettimeofday(&at, NULL) == 0);
        clock_t a = clock();
        // extract features
        extract(img, mask, kpts);
        // timing end
        clock_t b = clock();
        timeval bt;
        assert(gettimeofday(&bt, NULL) == 0);
        timeval abt;
        timersub(&bt, &at, &abt);
        // update statistics
        bm.time_cpu += 1000. * (b - a) / CLOCKS_PER_SEC; 
        bm.time_real += (1000. * abt.tv_sec + abt.tv_usec / 1000.); 
        bm.quantity += kpts.size();
        bm.n++;
        // save sample image
        if (i == 0) {
            bm.sample_img = imread(p.string(), 1);
            drawKeypoints(bm.sample_img, kpts);
        }
        i++;
    }
    cout << endl;
    return bm;
}

TEST_F(test_extractor, sift_surf_orb_benchmark) {
    if (!fast()) {
        // Read images
        bfs::path p(getenv("CLUTSEG_PATH"));
        bfs::path v = p / "ias_kinect_test_grounded_21";
        GroundTruth g = loadGroundTruthWithoutPoses((v / "ground-truth.txt").string());
        vector<bfs::path> image_paths;
        /* doesn't work for some ******* reason --- std::transform(g.begin(), g.end(),
            images.begin(), images.end(),
            boost::bind<string>(&std::pair<string, LabelSet>::first, _1)); 
            ---  anyways the following is even simpler */
        for (GroundTruth::iterator test_it = g.begin(); test_it != g.end(); test_it++) {
            image_paths.push_back(v / test_it->first);
        }

        benchmark_t sift = benchmark(SIFT(), image_paths);
        benchmark_t surf = benchmark(SURF(), image_paths);
        benchmark_t orb = benchmark(ORB(1500), image_paths);

        benchmark_t::header();
        sift.print("SIFT");
        surf.print("SURF");
        orb.print("ORB");

        imwrite("build/sift_surf_orb_benchmark_sift.png", sift.sample_img);
        imwrite("build/sift_surf_orb_benchmark_surf.png", surf.sample_img);
        imwrite("build/sift_surf_orb_benchmark_orb.png", orb.sample_img);
    }
}

// ORB response on n_features
// ---------------------------------------------------------------------------

size_t orb_response_n_features(const Mat & img, int n_features) {
    Mat mask = Mat::ones(img.size(), CV_8UC1);
    vector<KeyPoint> kpts;
    ORB orb = ORB(n_features);
    orb(img, mask, kpts);
    if (n_features == 0) {
        Mat canvas = img.clone();
        drawKeypoints(canvas, kpts);
        imwrite(str(boost::format("build/image-n-features-0-%d.png") % clock()), canvas);
    }
    return kpts.size();
}

TEST_F(test_extractor, orb_response_n_features) {
    map<string, Mat> imgs;
    imgs["assam_tea"] = imread("data/assam_tea.png", 0);
    imgs["clutter"] = imread("data/clutter.jpg", 0);
    imgs["house"] = imread("data/house.tiff", 0);
    imgs["mandrill"] = imread("data/mandrill.tiff", 0);
    // std::map guarantees order of keys (this is not the order of insertion)
    cout << "n_features ";
    for (map<string, Mat>::iterator img_it = imgs.begin(); img_it != imgs.end(); img_it++) {
        cout << boost::format("%10s ") % img_it->first;
        assert(img_it->second.empty() == false);
    }
    cout << endl;
    size_t sum = 0;
    // n_features = 0 leads to funny behaviour
    for (int n_features = 0; n_features <= 50000; n_features += 250) {
        if (n_features > 0) {
            cout << boost::format("%10d ") % n_features;
        }
        size_t s = 0;
        for (map<string, Mat>::iterator img_it = imgs.begin(); img_it != imgs.end(); img_it++) {
            size_t r = orb_response_n_features(img_it->second, n_features);
            if (n_features > 0) {
                cout << boost::format("%10d ") % r;
            }
            s += r;
        }
        if (n_features > 0) {
            cout << endl;
        }
        if (sum == s) {
            break;
        } else {
            sum = s;
        }
    }
}

// draw orb test locations
// ---------------------------------------------------------------------------

// int ORB::OrbPatterns::bit_pattern_31_ is private in OpenCV, the following
// bit_pattern_31_ array has been copied from
// opencv/modules/features2d/src/orb_patterns.hpp
int bit_pattern_31_[256*4] = {
8,-3, 9,5/*mean (0), correlation (0)*/,
4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

/* mean, corr, mean, corr, ... */
double bit_pattern_31_mean_corr[256*2] = {
0, 0,
1.12461e-05, 0.0437584,
3.37382e-05, 0.0617409,
5.62303e-05, 0.0636977,
0.000134953, 0.085099,
0.000528565, 0.0857175,
0.0188821, 0.0985774,
0.0363135, 0.0899616,
0.121806, 0.099849,
0.122065, 0.093285,
0.162787, 0.0942748,
0.21561, 0.0974438,
0.160583, 0.130064,
0.228171, 0.132998,
0.00997526, 0.145926,
0.198234, 0.143636,
0.0676226, 0.16689,
0.166847, 0.171682,
0.101215, 0.179716,
0.200641, 0.192279,
0.205106, 0.186848,
0.234908, 0.192319,
0.0709964, 0.210872,
0.0939834, 0.212589,
0.127778, 0.20866,
0.14783, 0.206356,
0.182141, 0.198942,
0.188237, 0.21384,
0.14865, 0.23571,
0.222312, 0.23324,
0.229082, 0.23389,
0.241577, 0.215286,
0.00338507, 0.251373,
0.131005, 0.257622,
0.152755, 0.255205,
0.182771, 0.244867,
0.186898, 0.23901,
0.226226, 0.258255,
0.0897886, 0.274827,
0.148774, 0.28065,
0.153048, 0.283063,
0.169523, 0.278248,
0.225337, 0.282851,
0.226687, 0.278734,
0.00693882, 0.305161,
0.0227283, 0.300181,
0.125517, 0.31089,
0.131748, 0.312779,
0.144827, 0.292797,
0.149202, 0.308918,
0.160909, 0.310013,
0.177755, 0.309394,
0.212337, 0.310315,
0.214429, 0.311933,
0.235807, 0.313104,
0.00494827, 0.344948,
0.0549145, 0.344675,
0.103385, 0.342715,
0.134222, 0.322922,
0.153284, 0.337061,
0.154881, 0.329257,
0.200967, 0.33312,
0.201518, 0.340635,
0.207805, 0.335631,
0.224438, 0.34504,
0.239361, 0.338053,
0.240744, 0.344322,
0.242949, 0.34145,
0.244028, 0.336861,
0.247571, 0.343684,
0.000697256, 0.357265,
0.00213675, 0.373827,
0.0126856, 0.373938,
0.0152497, 0.364237,
0.0299933, 0.345292,
0.0307242, 0.366299,
0.0534975, 0.368357,
0.099865, 0.372276,
0.117083, 0.364529,
0.126125, 0.369606,
0.130364, 0.358502,
0.131691, 0.375531,
0.160166, 0.379508,
0.167848, 0.353343,
0.183378, 0.371916,
0.228711, 0.371761,
0.247211, 0.364063,
0.249325, 0.378139,
0.000652272, 0.411682,
0.00248538, 0.392988,
0.0206815, 0.386106,
0.0364485, 0.410752,
0.0376068, 0.398374,
0.0424202, 0.405663,
0.0942645, 0.410422,
0.1074, 0.413224,
0.109256, 0.408646,
0.131691, 0.416076,
0.165081, 0.417569,
0.171874, 0.408471,
0.175146, 0.41296,
0.183682, 0.402956,
0.184672, 0.416125,
0.191487, 0.386696,
0.192668, 0.394771,
0.200157, 0.408303,
0.204588, 0.411762,
0.205904, 0.416294,
0.213237, 0.409306,
0.243444, 0.395069,
0.247672, 0.413392,
0.24774, 0.411416,
0.00213675, 0.454003,
0.0293635, 0.455368,
0.0404971, 0.457393,
0.0481107, 0.448364,
0.050641, 0.455019,
0.0525978, 0.44338,
0.0629667, 0.457096,
0.0653846, 0.445623,
0.0858749, 0.449789,
0.122402, 0.450201,
0.125416, 0.453224,
0.130128, 0.458724,
0.132467, 0.440133,
0.132692, 0.454,
0.135695, 0.455739,
0.142904, 0.446114,
0.146165, 0.451473,
0.147627, 0.456643,
0.152901, 0.455036,
0.167083, 0.459315,
0.173234, 0.454706,
0.18312, 0.433855,
0.185504, 0.443838,
0.185706, 0.451123,
0.188968, 0.455808,
0.191667, 0.459128,
0.193196, 0.458364,
0.196536, 0.455782,
0.1972, 0.450481,
0.199438, 0.458156,
0.211224, 0.449548,
0.211718, 0.440606,
0.213034, 0.443177,
0.234334, 0.455304,
0.235684, 0.443436,
0.237674, 0.452525,
0.23962, 0.444824,
0.248459, 0.439621,
0.249505, 0.456666,
0.00119208, 0.495466,
0.00372245, 0.484214,
0.00741116, 0.499854,
0.0208952, 0.499773,
0.0220085, 0.501609,
0.0233806, 0.496568,
0.0236505, 0.489719,
0.0268781, 0.503487,
0.0323324, 0.501938,
0.0399235, 0.494029,
0.0420153, 0.486579,
0.0548021, 0.484237,
0.0616622, 0.496642,
0.0627755, 0.498563,
0.0829622, 0.495491,
0.0843342, 0.487146,
0.0929937, 0.502315,
0.113327, 0.48941,
0.132119, 0.467268,
0.136269, 0.498771,
0.142173, 0.498714,
0.144141, 0.491973,
0.14892, 0.500782,
0.150371, 0.498211,
0.152159, 0.495547,
0.156152, 0.496925,
0.15749, 0.499222,
0.159211, 0.503821,
0.162427, 0.501907,
0.16652, 0.497632,
0.169141, 0.484474,
0.169456, 0.495339,
0.171457, 0.487251,
0.175, 0.500024,
0.175866, 0.497523,
0.178273, 0.501854,
0.181107, 0.494888,
0.190227, 0.482557,
0.196739, 0.496503,
0.19973, 0.499759,
0.204465, 0.49873,
0.209334, 0.49063,
0.211134, 0.503011,
0.212, 0.499414,
0.212168, 0.480739,
0.212731, 0.502523,
0.21327, 0.489786,
0.214159, 0.488246,
0.216993, 0.50287,
0.223639, 0.470502,
0.224089, 0.500852,
0.228666, 0.502629,
0.22906, 0.498305,
0.233378, 0.503825,
0.234323, 0.476692,
0.236392, 0.475462,
0.236842, 0.504132,
0.236977, 0.497739,
0.24314, 0.499398,
0.243297, 0.489447,
0.00155196, 0.553496,
0.00239541, 0.54297,
0.0034413, 0.544361,
0.003565, 0.551225,
0.00835583, 0.55285,
0.00885065, 0.540913,
0.0101552, 0.551085,
0.0102227, 0.533635,
0.0110211, 0.543121,
0.0113473, 0.550173,
0.0140913, 0.554774,
0.017049, 0.55461,
0.01778, 0.546921,
0.0224022, 0.549667,
0.029161, 0.546295,
0.0303081, 0.548599,
0.0355151, 0.523943,
0.0417904, 0.543395,
0.0487292, 0.542818,
0.0575124, 0.554888,
0.0594242, 0.544026,
0.0597391, 0.550524,
0.0608974, 0.55383,
0.065126, 0.552006,
0.074224, 0.546372,
0.0808592, 0.554875,
0.0883378, 0.551178,
0.0901035, 0.548446,
0.0949843, 0.554694,
0.0994152, 0.550979,
0.10045, 0.552714,
0.100686, 0.552594,
0.101091, 0.532394,
0.101147, 0.525576,
0.105263, 0.531498,
0.110785, 0.540491,
0.112798, 0.536582,
0.114181, 0.555793,
0.117431, 0.553763,
0.118522, 0.553452,
0.12094, 0.554785,
0.122582, 0.555825,
0.124978, 0.549846,
0.127002, 0.537452,
0.127148, 0.547401
};

/* TODO: this test is very drafty, needs review whether the ORB patterns have
 * been correctly interpreted. */
TEST_F(test_extractor, orb_draw_test_locations) {
    // Find extents of the test patterns
    int pmin = *min_element(bit_pattern_31_, bit_pattern_31_ + 256*4);
    int pmax = *max_element(bit_pattern_31_, bit_pattern_31_ + 256*4);
    int margin = 10;
    int scale = 25;
    Mat pat = Mat::zeros(2 * margin + scale * (pmax - pmin), 2 * margin + scale * (pmax - pmin), CV_32FC3);
    pat = Scalar::all(1.0);
    int xmin = pmax;
    int xmax = pmin;
    int ymin = pmax;
    int ymax = pmin;
    for (size_t i = 0; i < 256; i++) {
        int x1 = bit_pattern_31_[4*i + 0];
        int y1 = bit_pattern_31_[4*i + 1];
        int x2 = bit_pattern_31_[4*i + 2];
        int y2 = bit_pattern_31_[4*i + 3];
        // TODO: verify order of indexing
        int i1 = margin + 25 * (x1 - pmin);
        int j1 = margin + 25 * (y1 - pmin);
        int i2 = margin + 25 * (x2 - pmin);
        int j2 = margin + 25 * (y2 - pmin);
        // Random color:
        // Scalar c = Scalar(
        //      0.75 - double(0.75 * (1.0 * rand() / RAND_MAX)),
        //      0.75 - double(0.75 * (1.0 * rand() / RAND_MAX)),
        //      0.75 - double(0.75 * (1.0 * rand() / RAND_MAX)));
        // Color by correlation
        // Scalar c = Scalar::all(1.0 / bit_pattern_31_mean_corr[256*2-1] * bit_pattern_31_mean_corr[2*i + 1]);
        Scalar c = Scalar::all(0);
        circle(pat, Point(i1, j1), max(1, scale / 8), c); 
        circle(pat, Point(i2, j2), max(1, scale / 8), c); 
        line(pat, Point(i1, j1), Point(i2, j2), c);
        xmin = min(x1, xmin);
        xmin = min(x2, xmin);
        xmax = max(x1, xmax);
        xmax = max(x2, xmax);
        ymin = min(y1, ymin);
        ymin = min(y2, ymin);
        ymax = max(y1, ymax);
        ymax = max(y2, ymax);
    }
    cout << "xmin: " << xmin << endl;
    cout << "xmax: " << xmax << endl;
    cout << "ymin: " << ymin << endl;
    cout << "ymax: " << ymax << endl;
    imshow("ORB test locations", pat);
    waitKey(-1);
}
