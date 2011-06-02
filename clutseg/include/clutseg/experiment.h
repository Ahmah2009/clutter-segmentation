/**
 * Author: Julius Adorf
 */

#ifndef _EXPERIMENT_H_
#define _EXPERIMENT_H_

// TODO: responsibilities between experiment.h and paramsel.h are unclear
#include <boost/filesystem.hpp>
#include <set>
#include <string>
#include <tod/training/feature_extraction.h>
#include <tod/detecting/Parameters.h>

namespace clutseg {

    /** Cache entry */
    struct TrainFeatures {

        TrainFeatures();
        TrainFeatures(const std::string & train_set, const tod::FeatureExtractionParams & fe_params);

        std::string train_set;
        tod::FeatureExtractionParams fe_params;

        void generate();

        bool operator==(const TrainFeatures & rhs) const;
        bool operator!=(const TrainFeatures & rhs) const;
        bool operator<(const TrainFeatures & rhs) const;

    };

    /** Caches extracted features from training images in a two-level
     * directory. Each set of training features is uniquely defined by its original
     * training data set and the feature extraction parameters. This cache manager
     * is responsible for retrieving existing training feature sets, and for
     * determining whether new feature sets have to be generated.
     *
     * Extracting the features from training images is a very expensive step, it
     * shall not repated and this class allows to cache training features. Masking
     * and pose estimation is done up-front and does not have to be considered.
     * The cache directory is organized in an hierarchical manner. The first level
     * distinguishes between features that have been generated from different
     * training data. The second level distinguishes between features that have
     * been generated using different feature configurations.
     *
     * Example:
     * train_bases/
     *    tod_kinect_train/
     *    ias_kinect_train/
     *    ias_kinect_train_v2/
     *        71bcccd2efe711e112f0e4b8e1c2465a86133a6d/
     *        586105d85ee7102613a3c56ddf0be52475a40aed/
     *        635b0a4b2972d8d0a82f788da39a4f12a31ca92e/
     *            assam_tea/
     *                image_00000.png.f3d.yaml.gz
     *                image_00001.png.f3d.yaml.gz
     *                ...
     *            ...
     *            features.config.yaml
     *            config.txt
     *       ...
     *   ...
     *
     *
     * The names of the cache entries are sha1 hash values generated from
     * features.config.yaml files. Checking collisions is probably just
     * paranoid (less than 10**-20), but could also be done by comparing
     * requested feature configuration with the feature configuration loaded
     * from the training set.
     */
    class TrainFeaturesCache {

        public:

            TrainFeaturesCache();

            TrainFeaturesCache(const boost::filesystem::path & cache_dir);

            boost::filesystem::path trainFeaturesDir(const TrainFeatures & tr_feat);

            bool trainFeaturesExist(const TrainFeatures & tr_feat);

            float trainRuntime(const TrainFeatures & tr_feat);

            void addTrainFeatures(const TrainFeatures & tr_feat, bool consistency_check = true);

            bool trainFeaturesBlacklisted(const TrainFeatures & tr_feat);
            
            void blacklistTrainFeatures(const TrainFeatures & tr_feat);

        private:
            
            bool initialized_;
            
            boost::filesystem::path cache_dir_;

            std::set<TrainFeatures> blacklist_;

    };

    std::string sha1(const std::string & file);

    std::string sha1(const tod::FeatureExtractionParams & feParams);


    void readFeParams(const boost::filesystem::path & p, tod::FeatureExtractionParams & feParams);

    void writeFeParams(const boost::filesystem::path & p, const tod::FeatureExtractionParams & feParams);

    void readTodParams(const boost::filesystem::path & p, tod::TODParameters & todParams);

    void writeTodParams(const boost::filesystem::path & p, const tod::TODParameters & todParams);

    std::set<std::string> listTemplateNames(const boost::filesystem::path & dir);

}

#endif
