#include "textured_object_detection/ts_parameters.h"
#include <fstream>
#include "cv.h"
using namespace std;
using namespace cv;

TSParameters::TSParameters()
{
  detectorName = "FAST";
  descriptorName = "SURF";
  detectorThreshold = 5.0;
  nonmaxSuppression = false;
  minClusterSize = 10;
  clusterThreshold = 100;
  reprojectErrorThreshold = 2.0;
  minInliersCount = 10;
  knnNum = 30;
  ratioTestThreshold = 1.0;
  scale = 1.0;
  isRecalcDescriptors = false;
  minStddevFactor = 0.15;
}

bool TSParameters::readParameters(std::string path)
{
  FileStorage fs;
  fs.open(path, FileStorage::READ);

  if (!fs.isOpened())
    return false;

  fs["descriptorName"] >> descriptorName;
  fs["detectorName"] >> detectorName;
  fs["detectorThreshold"] >> detectorThreshold;
  fs["nonmaxSuppression"] >> nonmaxSuppression;
  fs["clusterThreshold"] >> clusterThreshold;
  fs["minClusterSize"] >> minClusterSize;
  fs["reprojectErrorThreshold"] >> reprojectErrorThreshold;
  fs["minInliersCount"] >> minInliersCount;
  fs["knnNum"] >> knnNum;
  fs["ratioTestThreshold"] >> ratioTestThreshold;
  fs["scale"] >> scale;
  fs["isRecalcDescriptors"] >> isRecalcDescriptors;
  fs["minStddevFactor"] >> minStddevFactor;

  fs.release();

  return true;
}

bool TSParameters::saveParameters(std::string path)
{
  FileStorage fs;
  fs.open(path, FileStorage::WRITE);

  fs << "detectorName" << detectorName;
  fs << "descriptorName" << descriptorName;
  fs << "detectorThreshold" << detectorThreshold;
  fs << "nonmaxSuppression" << nonmaxSuppression;
  fs << "minClusterSize" << minClusterSize;
  fs << "clusterThreshold" << clusterThreshold;
  fs << "reprojectErrorThreshold" << reprojectErrorThreshold;
  fs << "minInliersCount" << minInliersCount;
  fs << "knnNum" << knnNum;
  fs << "ratioTestThreshold" << ratioTestThreshold;
  fs << "scale" << scale;
  fs << "isRecalcDescriptors" << isRecalcDescriptors;
  fs << "minStddevFactor" << minStddevFactor;

  fs.release();

  return true;
}
