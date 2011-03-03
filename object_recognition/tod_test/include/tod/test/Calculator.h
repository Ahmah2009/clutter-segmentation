/*
 * Calculator.h
 *
 *  Created on: Feb 5, 2011
 *      Author: Alexander Shishkov
 */

#ifndef CALCULATOR_H_
#define CALCULATOR_H_

#include <string>
#include <opencv2/core/core.hpp>

#include <tod/test/ExtendedObjectInfo.h>
#include <tod/core/TrainingBase.h>

typedef map<string,int> ImageMap;

namespace tod
{
  struct CalculatorParameters
  {
    std::string logFilename;
    std::string trainDirectory;
    int verbose;
  };

  class Calculator
  {
  public:
    Calculator(const CalculatorParameters& params_);
    void calculate();
  private:
    CalculatorParameters params;

    std::string extractTestBasePath(const std::string& testFolder);
    void drawProjection(cv::Mat& out, const ExtendedObjectInfo& objectInfo,
                        const cv::Ptr<TexturedObject>& object);
    void calculateDetectionRate(cv::FileNode& testNode, TrainingBase& base);
    void generateGuessMask(const ExtendedObjectInfo& objectInfo, const TrainingBase& base,
                           int rows, int cols, cv::Mat& binMask);
    bool isInlier(const cv::Mat& mask, const cv::Mat& guessMask);
    void addToMap(ImageMap& map, const std::string& key);
    void getObjects(const std::string& testFolder, std::vector<std::string>& objects);
    int getImagesCount(std::string directory);
    int getCount(ImageMap& map);
    int getSum(ImageMap& map);
  };
}

#endif /* CALCULATOR_H_ */
