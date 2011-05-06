#ifndef FEATURE_PARAMS_H_TOD_
#define FEATURE_PARAMS_H_TOD_

#include <tod/core/common.h>
#include <opencv2/core/core.hpp>
#include <map>

namespace tod {

    struct FeatureExtractionParams : public Serializable {

      std::string detector_type;
      std::string descriptor_type;
      std::string extractor_type;
      std::map<std::string, double> detector_params;
      std::map<std::string, double> extractor_params;
      //serialization
      virtual void write(cv::FileStorage& fs) const;
      virtual void read(const cv::FileNode& fn);

      static FeatureExtractionParams CreateSampleParams();
      static const std::string YAML_NODE_NAME;
    };

}

#endif

