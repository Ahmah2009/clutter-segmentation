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

    /** Caches extracted features from training images in a two-level
     * directory. Each set of training features is uniquely defined by its original
     * training data set and the feature extraction parameters. This cache manager
     * is responsible for retrieving existing training feature sets, and for
     * determining whether new feature sets have to be generated.
     */
    class TrainCache {

        public:

            TrainCache(const std::string & cache_dir);

            std::string trainFeaturesDir(const std::string & train_set, const tod::FeatureExtractionParams & feParams);

            bool trainFeaturesExist(const std::string & train_set, const tod::FeatureExtractionParams & feParams);

        private:

            const std::string cache_dir_;

    };

    std::string sha1(const std::string & file);

    std::string sha1(const tod::FeatureExtractionParams & feParams);

}

