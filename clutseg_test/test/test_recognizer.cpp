/*
 * Author: Julius Adorf
 *
 * This test helps understand tod_detecting pipeline by validating assumptions,
 * and thus generate hard facts. It is basically copied from recognizer.cpp in
 * tod_detecting.
 */

#include "test.h"

#include "clutseg/pose.h"
#include "clutseg/viz.h"

#include <unistd.h>
#include <stdlib.h>
#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tod/detecting/Loader.h>
#include <tod/detecting/Parameters.h>
#include <tod/detecting/Recognizer.h>
#include <tod/training/feature_extraction.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <set>

#define foreach BOOST_FOREACH
using std::list;
using std::string;
using std::istream;
using namespace cv;
using namespace tod;
using namespace std;
using namespace clutseg;

namespace po = boost::program_options;

namespace {
    struct Options
    {
        std::string imageFile;
        std::string baseDirectory;
        std::string config;
        TODParameters params;
        int verbose;
        int mode;
    };
}

void drawProjections(const Mat & image, int id,
                     const vector < Guess > &guesses,
                     const TrainingBase & base, const Options & opts)
{
    if (guesses.empty())
        return;
    Mat drawImg, correspondence;
    image.copyTo(drawImg);
    namedWindow("proj", 1);
    namedWindow("correspondence", CV_WINDOW_KEEPRATIO);
    foreach(const Guess & guess, guesses) {
        guess.draw(drawImg, 0, ".");
        Mat temp, small;
        drawImg.copyTo(temp);
        resize(temp, small, Size(640, 480), CV_INTER_LINEAR);
        small.copyTo(drawImg);
        imshow("proj", drawImg);
        guess.draw(correspondence, 1, opts.baseDirectory);
        if (!correspondence.empty())
            imshow("correspondence", correspondence);
        waitKey(0);
    }
}

TEST(Recognizer, TestRun)
{
    string p(getenv("CLUTSEG_PATH"));
    Options opts;
    opts.baseDirectory = p + "/ias_kinect_train";
    opts.imageFile = p + "/ias_kinect_test_all/all01/image_00014.jpg";
    opts.config = p + "/ias_kinect_train/config.yaml";
    opts.verbose = 0;
    opts.mode = KINECT;

    FileStorage fs;
    fs = FileStorage(opts.config, FileStorage::READ);
    opts.params.read(fs[TODParameters::YAML_NODE_NAME]);
    
    tod::Loader loader(opts.baseDirectory);
    vector < cv::Ptr < TexturedObject > >objects;
    loader.readTexturedObjects(objects);

    // [julius] Assumption: All 4 training objects are represented by one
    // TexturedObject.
    EXPECT_EQ(4, objects.size());
    // [julius] Assumption: The Loader does not assign identifiers to objects
    foreach(const cv::Ptr<TexturedObject> object, objects) {
        EXPECT_EQ(-1, object->id); 
    }

    TrainingBase base(objects);

    // [julius] These assertions prove that the TrainingBase assigns numeric
    // identifiers to the textured objects. The kind of bloated TrainingBase
    // class introduces another layer of indirection that does not seem to
    // serve any other purpose than code obfuscation.
    foreach(const cv::Ptr<TexturedObject> object, objects) {
        EXPECT_NE(-1, object->id); 
    }
 
    // [julius] Assumption: The identifiers in TrainingBase are the same as
    // those given to the textured objects themselves. So, this duplicated
    // information is at least consistent. 
    set<int> objectIds;
    base.getObjectIds(objectIds);
    foreach(const int objectId, objectIds) {
        EXPECT_EQ(objectId, base.getObject(objectId)->id);
    }
    // [julius] Assumption: The TrainingBase just assigns identifiers according
    // to the index of a training subject in 'objects'. So the generated
    // identifiers are in the set [0, objects.size()-1] though the comments in
    // TrainingBase indicate otherwise.
    for (size_t i = 0; i < objects.size(); i++) {
        EXPECT_TRUE(objectIds.end() != objectIds.find(i));
        cout << boost::format("Mapping objects[%d] to object id %d") % i % (*objectIds.find(i)) << endl;
    }

    Ptr<FeatureExtractor> extractor =
        FeatureExtractor::create(opts.params.feParams);
    Features2d test;
    // [julius] The image is read in grayscale, see OpenCV documentation-
    test.image = imread(opts.imageFile, 0);

    extractor->detectAndExtract(test);
    
    std::cout << "Extracted " << test.keypoints.
        size() << " points" << std::endl;

    cv::Ptr<Matcher> matcher = Matcher::create(opts.params.matcherParams);
    // [julius] This adds all keypoint descriptors from the training images to
    // the matcher. See OpenCV's DescriptorMatcher.
    matcher->add(base);

    cv::Ptr<Recognizer> recognizer;
    if (opts.mode == TOD) {
        recognizer =
            new TODRecognizer(&base, matcher, &opts.params.guessParams,
                              opts.verbose, opts.baseDirectory,
                              opts.params.clusterParams.maxDistance);
    } else if (opts.mode == KINECT) {
        recognizer =
            new KinectRecognizer(&base, matcher, &opts.params.guessParams,
                                 opts.verbose, opts.baseDirectory);
    } else {
        std::cout << "Invalid mode option!" << std::endl;
        ASSERT_TRUE(false);
    }

    vector < Guess > guesses;
    // [julius] This is the key function call in this whole main method.
    recognizer->match(test, guesses);

    // [julius] Assume that there are about 10*<number of test keypoints>
    // matches found.
    size_t totalMatchCnt = 0;
    foreach(int objectId, objectIds) {
        vector<DMatch> matches;
        matcher->getObjectMatches(objectId, matches);
        totalMatchCnt += matches.size();
    }
    EXPECT_LT(2 * test.keypoints.size(), totalMatchCnt);
    EXPECT_LT(totalMatchCnt, 4 * test.keypoints.size());
    
    foreach(const Guess & guess, guesses) {
        cout << "Object name = " << guess.getObject()->name << ", imageId = ";
        // Get the used image indices
        set < unsigned int >image_indices;
        BOOST_FOREACH(unsigned int inlier_index, guess.inliers)
            image_indices.insert(guess.image_indices_[inlier_index]);
        BOOST_FOREACH(unsigned int image_index, image_indices)
            cout << image_index << " ";
        cout << std::endl;

        Mat canvas = test.image.clone();
        drawPose(canvas, guess.aligned_pose(), guess.getObject()->observations[0].camera());
        imshow(guess.getObject()->name, canvas);
        waitKey(0);
    }

    if (enableUI) {
       drawProjections(test.image, -1, guesses, base, opts);
    }
}

