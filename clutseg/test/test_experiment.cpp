/**
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/clutseg.h"
#include "clutseg/common.h"
#include "clutseg/modelbase.h"
#include "clutseg/flags.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <iostream>

using namespace boost::filesystem;
using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

struct test_experiment : public ::testing::Test {

    void SetUp() {
        readFeParams("./data/features.config.yaml", feParams);
        // Generated from file.
        feParamsSha1 = "b8bb41a305d5616c97f54efc0edb63af561fe342";
        // Arbitrary.
        train_set = "ias_kinect_train_v2";
        tr_feat.fe_params = feParams;
        tr_feat.train_set = train_set;
        cache_dir = "build/train_cache";
        // corresponds to feParams 
        feat_dir = bfs::path(cache_dir) / train_set / feParamsSha1;
        bfs::create_directories(cache_dir);
        cache = ModelbaseCache(cache_dir);
    }

    void TearDown() {
        bfs::remove_all(cache_dir);
    }

    FeatureExtractionParams feParams;
    string feParamsSha1;
    string train_set;
    Modelbase tr_feat;
    string cache_dir;
    bfs::path feat_dir;
    ModelbaseCache cache;

};

TEST_F(test_experiment, gen_hash_from_file) {
    EXPECT_EQ("2605fd43e5192a2e49476e5099f8ea6e2973866b", sha1("./data/camera.yml"));
}

TEST_F(test_experiment, gen_hash_from_feparams) {
    EXPECT_EQ(feParamsSha1, sha1(feParams));
    EXPECT_EQ(feParamsSha1, sha1(tr_feat.fe_params));
}


TEST_F(test_experiment, gen_hash_from_empty_feparams) {
    FeatureExtractionParams feParams;
    EXPECT_EQ("f8767180bfcd0654f4ffe9514e94e6d51324e3f6", sha1(feParams));
}

TEST_F(test_experiment, file_has_same_hash_as_feparams) {
    // Ensure that the hash generated from a file is the same as the hash of
    // the parameteres retrieved from the file. This does not hold if there
    // are any comments in the features.config.yaml. Also, if the parameters
    // change, all the relationships in the cache get lost. Anyways, in this
    // case we can read in the features.config.yaml files, write them back
    // to the cache directories, regenerate the SHA1 hashes and rename the
    // cache directories to recover.
    EXPECT_EQ(sha1("./data/features.config.yaml"), sha1(feParams));
}

TEST_F(test_experiment, train_features_equal) {
    FeatureExtractionParams feParams1;
    FeatureExtractionParams feParams2;
    readFeParams("./data/features.config.yaml", feParams1);
    readFeParams("./data/features.config.yaml", feParams2);
    Modelbase tr_feat_1("train_set", feParams1);
    Modelbase tr_feat_2("train_set", feParams2);
    EXPECT_TRUE(tr_feat_1 == tr_feat_2);
}

TEST_F(test_experiment, train_features_not_equal) {
    FeatureExtractionParams feParams1;
    FeatureExtractionParams feParams2;
    readFeParams("./data/features.config.yaml", feParams1);
    readFeParams("./data/features.config.yaml", feParams2);
    feParams2.detector_type = "STAR";
    Modelbase tr_feat_1("train_set", feParams1);
    Modelbase tr_feat_2("train_set", feParams2);
    EXPECT_TRUE(tr_feat_1 != tr_feat_2);
}


TEST_F(test_experiment, train_features_dir) {
    EXPECT_EQ(feat_dir.string(), cache.modelbaseDir(tr_feat)); 
}

TEST_F(test_experiment, train_features_exist) {
    ModelbaseCache cache(cache_dir);
    EXPECT_FALSE(cache.modelbaseExist(tr_feat)); 
    bfs::create_directories(feat_dir);
    EXPECT_TRUE(cache.modelbaseExist(tr_feat));
}

TEST_F(test_experiment, add_train_features_fail_if_already_exist) {
    cache.addModelbase(tr_feat, false);
    EXPECT_TRUE(cache.modelbaseExist(tr_feat));
    try {
        cache.addModelbase(tr_feat);
        EXPECT_TRUE(false);
    } catch (...) {
        
    }
}

TEST_F(test_experiment, add_train_features) {
    EXPECT_FALSE(cache.modelbaseExist(tr_feat));
    cache.addModelbase(tr_feat, false);
    EXPECT_TRUE(cache.modelbaseExist(tr_feat));
    EXPECT_TRUE(bfs::exists(feat_dir / "assam_tea" / "image_00000.png.f3d.yaml.gz"));
    EXPECT_TRUE(bfs::exists(feat_dir / "haltbare_milch" / "image_00025.png.f3d.yaml.gz"));
    EXPECT_TRUE(bfs::exists(feat_dir / "icedtea" / "image_00012.png.f3d.yaml.gz"));
    EXPECT_TRUE(bfs::exists(feat_dir / "jacobs_coffee" / "image_00032.png.f3d.yaml.gz"));
    EXPECT_TRUE(bfs::exists(feat_dir / "features.config.yaml"));
}

TEST_F(test_experiment, gen_and_use_train_features) {
    SKIP_IF_FAST

    cache.addModelbase(tr_feat, false);
    Modelbase new_tr_feat;
    new_tr_feat.train_set = tr_feat.train_set;
    new_tr_feat.fe_params = FeatureExtractionParams::CreateSampleParams();
    new_tr_feat.fe_params.detector_params["threshold"] = 40;
    bfs::path p = bfs::path(getenv("CLUTSEG_PATH"));
    writeFeParams((p / train_set / "features.config.yaml").string(), new_tr_feat.fe_params);

    new_tr_feat.generate();
    cache.addModelbase(new_tr_feat);
    EXPECT_NE(sha1(new_tr_feat.fe_params), sha1(tr_feat.fe_params));
    EXPECT_NE(cache.modelbaseDir(new_tr_feat), cache.modelbaseDir(tr_feat));

    FileFlag dirty(p / train_set / "dirty.flag");
    EXPECT_FALSE(dirty.exists());

    // TODO: load configs that make sense
    Clutsegmenter segmenter(
        cache.modelbaseDir(new_tr_feat).string(),
            (p / "ias_kinect_train/config.yaml").string(),
            (p / "ias_kinect_train/config.yaml").string()
        );
    set<string> tNames = segmenter.getTemplateNames();
    EXPECT_EQ(1, tNames.count("assam_tea"));
    EXPECT_EQ(1, tNames.count("haltbare_milch"));
    EXPECT_EQ(1, tNames.count("icedtea"));
    EXPECT_EQ(1, tNames.count("jacobs_coffee"));
    EXPECT_EQ(4, tNames.size());
}

TEST_F(test_experiment, list_template_names) {
    bfs::path p = bfs::path(getenv("CLUTSEG_PATH"));
    set<string> tNames = listTemplateNames(p / "ias_kinect_train_v2");
    EXPECT_EQ(1, tNames.count("assam_tea"));
    EXPECT_EQ(1, tNames.count("haltbare_milch"));
    EXPECT_EQ(1, tNames.count("icedtea"));
    EXPECT_EQ(1, tNames.count("jacobs_coffee"));
    EXPECT_EQ(4, tNames.size());
}

TEST_F(test_experiment, blacklist) {
    ASSERT_FALSE(cache.modelbaseBlacklisted(tr_feat));
    cache.blacklistModelbase(tr_feat);
    ASSERT_TRUE(cache.modelbaseBlacklisted(tr_feat));
}
