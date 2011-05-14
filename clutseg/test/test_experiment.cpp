/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <iostream>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

struct ExperimentTest : public ::testing::Test {

    void SetUp() {
        FileStorage fs("./data/features.config.yaml", FileStorage::READ);
        feParams.read(fs[FeatureExtractionParams::YAML_NODE_NAME]);
        fs.release();

        // Generated from file.
        feParamsSha1 = "b8bb41a305d5616c97f54efc0edb63af561fe342";

        // Arbitrary.
        train_set = "ias_kinect_train_v2";

        train_features.fe_params = feParams;
        train_features.train_set = train_set;

        cache_dir = "build/train_cache";
        
        // corresponds to feParams 
        features_dir = cache_dir + "/" + train_set + "/" + feParamsSha1;

        boost::filesystem::remove_all(cache_dir);
        boost::filesystem::create_directories(cache_dir);

        cache = TrainFeaturesCache(cache_dir);
    }

    FeatureExtractionParams feParams;
    string feParamsSha1;
    string train_set;
    TrainFeatures train_features;
    string cache_dir;
    string features_dir;
    TrainFeaturesCache cache;

};

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

TEST_F(ExperimentTest, GenerateHashFromFile) {
    EXPECT_EQ("2605fd43e5192a2e49476e5099f8ea6e2973866b", sha1("./data/camera.yml"));
}

TEST_F(ExperimentTest, GenerateHashFromFeatureExtractionParams) {
    EXPECT_EQ(feParamsSha1, sha1(feParams));
    EXPECT_EQ(feParamsSha1, sha1(train_features.fe_params));
}


TEST_F(ExperimentTest, GenerateHashFromEmptyFeatureExtractionParams) {
    FeatureExtractionParams feParams;
    EXPECT_EQ("f8767180bfcd0654f4ffe9514e94e6d51324e3f6", sha1(feParams));
}

TEST_F(ExperimentTest, FileHasSameHashAsFeatureExtractionParams) {
    // Ensure that the hash generated from a file is the same as the hash of
    // the parameteres retrieved from the file. This does not hold if there
    // are any comments in the features.config.yaml. Also, if the parameters
    // change, all the relationships in the cache get lost. Anyways, in this
    // case we can read in the features.config.yaml files, write them back
    // to the cache directories, regenerate the SHA1 hashes and rename the
    // cache directories to recover.
    EXPECT_EQ(sha1("./data/features.config.yaml"), sha1(feParams));
}

TEST_F(ExperimentTest, TestTrainFeaturesDir) {
    EXPECT_EQ(features_dir, cache.trainFeaturesDir(train_features)); 
}

TEST_F(ExperimentTest, TestTrainFeaturesExist) {
    TrainFeaturesCache cache(cache_dir);
    EXPECT_FALSE(cache.trainFeaturesExist(train_features)); 
    boost::filesystem::create_directories(features_dir);
    EXPECT_TRUE(cache.trainFeaturesExist(train_features));
}

TEST_F(ExperimentTest, AddTrainFeaturesFailIfAlreadyExist) {
    cache.addTrainFeatures(train_features, false);
    EXPECT_TRUE(cache.trainFeaturesExist(train_features));
    try {
        cache.addTrainFeatures(train_features);
        EXPECT_TRUE(false);
    } catch (...) {
        
    }
}

TEST_F(ExperimentTest, AddTrainFeatures) {
    EXPECT_FALSE(cache.trainFeaturesExist(train_features));
    cache.addTrainFeatures(train_features, false);
    EXPECT_TRUE(cache.trainFeaturesExist(train_features));
    EXPECT_TRUE(boost::filesystem::exists(features_dir + "/assam_tea/image_00000.png.f3d.yaml.gz"));
    EXPECT_TRUE(boost::filesystem::exists(features_dir + "/haltbare_milch/image_00025.png.f3d.yaml.gz"));
    EXPECT_TRUE(boost::filesystem::exists(features_dir + "/icedtea/image_00012.png.f3d.yaml.gz"));
    EXPECT_TRUE(boost::filesystem::exists(features_dir + "/jacobs_coffee/image_00032.png.f3d.yaml.gz"));
    EXPECT_TRUE(boost::filesystem::exists(features_dir + "/features.config.yaml"));
}

TEST_F(ExperimentTest, GenerateTrainFeatures) {
    cache.addTrainFeatures(train_features, false);
    TrainFeatures new_train_features;
    new_train_features.train_set = train_features.train_set;
    new_train_features.fe_params = FeatureExtractionParams::CreateSampleParams();
    new_train_features.fe_params.detector_params["threshold"] = 40;

    cv::FileStorage fs(string(getenv("CLUTSEG_PATH")) + "/" + train_set + "/features.config.yaml", cv::FileStorage::WRITE);
    fs << FeatureExtractionParams::YAML_NODE_NAME;
    new_train_features.fe_params.write(fs);
    fs.release();

    new_train_features.generate();
    cache.addTrainFeatures(new_train_features);
    EXPECT_NE(sha1(new_train_features.fe_params), sha1(train_features.fe_params));
    EXPECT_NE(cache.trainFeaturesDir(new_train_features), cache.trainFeaturesDir(train_features));
} // TODO: check whether stuff has really been copied
  // TODO: check whether feature configuration has been correctly stored
    // FIXME: get rid of annoying bug in detector
   // TODO: think about flags that allow the system to interrupt cleanly or mark things as dirty
    // TODO: think about some logging system maybe in boost

// TODO: make sha1 a member function of TrainFeatures

