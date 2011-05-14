/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"

#include <gtest/gtest.h>
#include <iostream>

using namespace clutseg;
using namespace std;

struct ExperimentTest : public ::testing::Test {

    void SetUp() {
        if (!loaded) {
             segmenter = ClutSegmenter(
                string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train",
                string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml",
                string(getenv("CLUTSEG_PATH")) + "/ias_kinect_train/config.yaml"
            );
            loaded = true;
        }
    }

    static ClutSegmenter segmenter;
    static bool loaded;

};

ClutSegmenter ExperimentTest::segmenter;
bool ExperimentTest::loaded;

TEST_F(ExperimentTest, ExtractFeatures) {
    // Given an experiment setup, we need to extract the features from the
    // training images.  Since this is an extraordinarily expensive step, it
    // shall not repated and we therefore cache the training features. Masking
    // and pose estimation is done up-front and does not have to be considered.
    // We have a directory that contains the training images, masks and poses.
    // We need to generate a training base for a given feature extraction
    // parameter set.
    
    // Inputs
    // - a training base
    // - a feature configuration
    // - a cache directory 

    // The cache directory must be well-organized. It shall be organized in an
    // hierarchical manner.  First, we need to distinguish between training
    // bases that have been generated from different training data. Then we
    // need to distinguish by feature configuration. That results in a
    // two-level filesystem hierarchy.

    // train_bases/
    //      tod_kinect_train/
    //      ias_kinect_train/
    //      ias_kinect_train_v2/
    //          71bcccd2efe711e112f0e4b8e1c2465a86133a6d/
    //          586105d85ee7102613a3c56ddf0be52475a40aed/
    //          71bcccd2efe711e112f0e4b8e1c2465a86133a6d/
    //          fb2ecb0b83813940380058ae05e7136f6be2b044/
    //          635b0a4b2972d8d0a82f788da39a4f12a31ca92e/
    //              features.config.yaml
    //              image_00000.png.f3d.yaml.gz
    //              image_00001.png.f3d.yaml.gz
    //              ...
    //          ...
    //      ...

    // The second level depends on the feature configuration. The name of the
    // training bases are sha1 hash values taken from features.config.yaml.
    // Checking collisions is probably just paranoid (less than 10**-20), but
    // could also be done by comparing requested feature configuration with the
    // feature configuration loaded from the training set.

    // The feature configuration file shall be generated from a database entry
    // and shall be quite stable in order to make it likely that the same
    // feature configuration will generate the same hash.

    // The generation of the features can be mostly done via the supplied tools
    // in tod_training. Instead of calling them from a shell, we can simply
    // call their main methods with appropriate parameters. The directory
    // containing the training data (which was formerly intended by tod_* also
    // to be the training base). The second step is to actually copy those
    // f3d.yaml.gz files over to the cache.

    EXPECT_TRUE(false); // acceptance?
}

TEST_F(ExperimentTest, GenerateHashFromFile) {
    EXPECT_TRUE(false);
}

TEST_F(ExperimentTest, GenerateHashFromFeatureExtractionParams) {
    EXPECT_TRUE(false);
}

TEST_F(ExperimentTest, FileHasSameHashAsFeatureExtractionParams) {
    EXPECT_TRUE(false);
}

