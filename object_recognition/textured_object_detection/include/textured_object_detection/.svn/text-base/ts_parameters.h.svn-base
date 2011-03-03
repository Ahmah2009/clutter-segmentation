#ifndef TS_PARAMETERS_H_
#define TS_PARAMETERS_H_
#include <string>

struct TSParameters
{
  std::string detectorName;
  float detectorThreshold;
  bool nonmaxSuppression;
  std::string descriptorName;
  int minClusterSize;
  int clusterThreshold;
  float reprojectErrorThreshold;
  int minInliersCount;
  int knnNum;
  float ratioTestThreshold;
  float scale;
  bool isRecalcDescriptors;
  float minStddevFactor;

  TSParameters();

  bool readParameters(std::string path);
  bool saveParameters(std::string path);
};


#endif /* TS_PARAMETERS_H_ */
