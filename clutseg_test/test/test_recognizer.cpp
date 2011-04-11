/*
 * Author: Julius Adorf
 *
 * This test helps understand tod_detecting pipeline by validating assumptions,
 * and thus generate hard facts. It is basically derived from recognizer.cpp in
 * tod_detecting.
 */

#include "test.h"
#include <gtest/gtest.h>
#include <unistd.h>
#include <stdlib.h>
#include <boost/format.hpp>

#include "tod/detecting/Loader.h"
#include "tod/detecting/Parameters.h"
#include "tod/detecting/Recognizer.h"
#include "tod/training/feature_extraction.h"

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <set>

#define foreach BOOST_FOREACH
typedef std::pair < int, int >idx_pair_t;
using std::list;
using std::string;
using std::istream;
using namespace cv;
using namespace tod;

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
    opts.imageFile = p + "/ias_kinect_train/haltbare_milch/image_00000.png";
    opts.config = p + "/ias_kinect_train/config.yaml";
    opts.verbose = 2;
    opts.mode = 1;

    FileStorage fs;
    fs = FileStorage(opts.config, FileStorage::READ);
    opts.params.read(fs[TODParameters::YAML_NODE_NAME]);
    
    // Validate assumptions on option values
    EXPECT_EQ(KINECT, opts.mode);
    EXPECT_EQ(2, opts.verbose);

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

    Ptr < FeatureExtractor > extractor =
        FeatureExtractor::create(opts.params.feParams);
    Features2d test;
    // [julius] The image is read in grayscale, see OpenCV documentation-
    test.image = imread(opts.imageFile, 0);

    extractor->detectAndExtract(test);
    
    std::cout << "Extracted " << test.keypoints.
        size() << " points" << std::endl;

    // [julius] The naming shows that the programmer inteded to use the
    // RatioTestMatcher though the used Matcher actually depends on the
    // parameter configuration and as such the name is rather confusing. 
    cv::Ptr < Matcher > rtMatcher =
        Matcher::create(opts.params.matcherParams);
    // [julius] This adds all keypoint descriptors from the training images to
    // the matcher. See OpenCV's DescriptorMatcher.
    rtMatcher->add(base);

    cv::Ptr < Recognizer > recognizer;
    if (opts.mode == TOD) {
        recognizer =
            new TODRecognizer(&base, rtMatcher, &opts.params.guessParams,
                              opts.verbose, opts.baseDirectory,
                              opts.params.clusterParams.maxDistance);
    } else if (opts.mode == KINECT) {
        recognizer =
            new KinectRecognizer(&base, rtMatcher, &opts.params.guessParams,
                                 opts.verbose, opts.baseDirectory);
    } else {
        std::cout << "Invalid mode option!" << std::endl;
        ASSERT_TRUE(false);
    }

    vector < tod::Guess > foundObjects;
    // [julius] This is the key function call in this whole main method.
    recognizer->match(test, foundObjects);

    foreach(const tod::Guess & guess, foundObjects) {
        std::cout << "Object name = " << guess.getObject()->
            name << ", imageId = ";
        // Get the used image indices
        std::set < unsigned int >image_indices;
        BOOST_FOREACH(unsigned int inlier_index, guess.inliers)
            image_indices.insert(guess.image_indices_[inlier_index]);
        BOOST_FOREACH(unsigned int image_index, image_indices)
            std::cout << image_index << " ";
        std::cout << std::endl;
    }

}

