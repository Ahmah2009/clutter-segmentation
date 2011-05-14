// TODO: ifdefs

/**
 * Author: Julius Adorf
 *
 * This module defines functionality for evaluating a system learnt on a
 * specific parameter set on testing set.
 */

#include <string>
#include <tod/training/feature_extraction.h>

namespace clutseg {

    /** Cache entry */
    struct TrainFeatures {

        std::string train_set;
        tod::FeatureExtractionParams fe_params;

        void generate();

    };

    /** Caches extracted features from training images in a two-level
     * directory. Each set of training features is uniquely defined by its original
     * training data set and the feature extraction parameters. This cache manager
     * is responsible for retrieving existing training feature sets, and for
     * determining whether new feature sets have to be generated.
     */
    class TrainFeaturesCache {

        public:

            #ifdef TEST
                TrainFeaturesCache();
            #endif

            TrainFeaturesCache(const std::string & cache_dir);

            std::string trainFeaturesDir(const TrainFeatures & train_features);

            bool trainFeaturesExist(const TrainFeatures & train_features);

            void addTrainFeatures(const TrainFeatures & train_features, bool consistency_check = true);

        private:

            std::string cache_dir_;

    };

    std::string sha1(const std::string & file);

    std::string sha1(const tod::FeatureExtractionParams & feParams);


    void readFeParams(const std::string & path, tod::FeatureExtractionParams & feParams);

    void writeFeParams(const std::string & path, const tod::FeatureExtractionParams & feParams);

}

