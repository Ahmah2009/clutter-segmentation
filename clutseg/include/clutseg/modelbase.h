/*
 * Author: Julius Adorf
 */

#ifndef _EXPERIMENT_H_
#define _EXPERIMENT_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <set>
    #include <string>
    #include <tod/training/feature_extraction.h>
    #include <tod/detecting/Parameters.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /** \brief A modelbase. 
     *
     * A modelbase contains the models of textured objects. An instance of this
     * class is merely a specification of how to create the models of an
     * object, given the data in Modelbase::train_set (images, point clouds,
     * masks, ...) and the parameters to use for extracting model features
     * (Modelbase::fe_params).
     */
    struct Modelbase {

        /** \brief Dummy constructor. Returns an invalid modelbase. */
        Modelbase();

        Modelbase(const std::string & train_set, const tod::FeatureExtractionParams & fe_params);

        std::string train_set;
        tod::FeatureExtractionParams fe_params;
    
        /**
         * \brief Generate the modelbase from the data and the feature
         * extraction parameters.
         *
         * The generated model features are stored in the directory specified
         * by Modelbase::train_set. It is the responsibility of the cache
         * manager (ModelbaseCache) to transfer the model features into the
         * cache.
         */
        void generate();

        /** \brief Two modelbases are equal if and only if they have been created from the 
         * same Modelbase::train_set directory and have the same feature extraction parameters. */
        bool operator==(const Modelbase & rhs) const;

        /** \brief See Modelbase::operator== */
        bool operator!=(const Modelbase & rhs) const;

        /** \brief Compares modelbases by train_set and by the SHA1 of the
         * feature extraction parameters. */
        bool operator<(const Modelbase & rhs) const;

    };

    /**
     * \brief Caches modelbases.
     *
     * Caches extracted features from training images in a two-level
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
    class ModelbaseCache {

        public:

            ModelbaseCache();

            ModelbaseCache(const boost::filesystem::path & cache_dir);

            boost::filesystem::path modelbaseDir(const Modelbase & tr_feat);

            bool modelbaseExist(const Modelbase & tr_feat);

            float trainRuntime(const Modelbase & tr_feat);

            void addModelbase(const Modelbase & tr_feat, bool consistency_check = true);

            bool modelbaseBlacklisted(const Modelbase & tr_feat);
            
            void blacklistModelbase(const Modelbase & tr_feat);

        private:
            
            bool initialized_;
            
            boost::filesystem::path cache_dir_;

            std::set<Modelbase> blacklist_;

    };

    /** \brief Computes the SHA1 hashcode for a file. */
    std::string sha1(const std::string & file);

    /**
     * \brief Computes the SHA1 hashcode for a set of feature extraction parameters.
     *
     * Writes the feature extraction parameters to a file, and then returns the SHA1
     * hashcode of the file.
     *
     * @see clutseg::sha1.
     */
    std::string sha1(const tod::FeatureExtractionParams & feParams);

    /**
     * \brief Read the feature extraction parameters from a YAML file.
     *
     * See clutseg/data/sps.features.config.yaml for an example file.
     *
     * @see writeFeParams
     */
    void readFeParams(const boost::filesystem::path & p, tod::FeatureExtractionParams & feParams);

    /**
     * \brief Write the feature extraction parameters to a YAML file.
     *
     * See clutseg/data/sps.features.config.yaml for an example file.
     *
     * @see readFeParams
     */
    void writeFeParams(const boost::filesystem::path & p, const tod::FeatureExtractionParams & feParams);

    /**
     * \brief Read the parameters for tod_* from a YAML file.
     *
     * See clutseg/data/sps.detect.config.yaml or
     * clutseg/data/sps.refine.config.yaml for examples.
     *
     * @see writeTodParams
     */
    void readTodParams(const boost::filesystem::path & p, tod::TODParameters & todParams);

    /**
     * \brief Write the parameters for tod_* to a YAML file.
     *
     * @see writeTodParams
     */
    void writeTodParams(const boost::filesystem::path & p, const tod::TODParameters & todParams);

    /** \brief List the names of the objects in a modelbase.
     *
     * Lists the directories. Each directory in the modelbase directory is assumed to be the name
     * of an object in the modelbase.
     */
    std::set<std::string> listTemplateNames(const boost::filesystem::path & dir);

}

#endif
