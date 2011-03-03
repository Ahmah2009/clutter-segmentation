/*
 * ObjectConverter.h
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#ifndef OBJECTCONVERTER_H_
#define OBJECTCONVERTER_H_
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tod/core/Features3d.h>

namespace tod
{
  class ObjectConverter
  {
  public:
    virtual ~ObjectConverter();
    ObjectConverter(const std::string& objectName, int imageCount,
                    const std::string& basePath, const std::string& resultPath);
    void convert();

  private:
    std::string name;
    std::string path;
    std::string resPath;
    int count;

    void loadPointsIndices(const std::string& filename, std::vector<uint16_t>& indices);
    void loadDescriptors(const std::string& filename, cv::Mat& descriptors);
    void loadKeypoints(const std::string& filename, std::vector<cv::KeyPoint>& keypoints);
    std::string generatePath(const std::string& filename, bool useResPath = false);
    void fillFeatures2d(Features2d& f2d, const std::string& digit, const std::string& filename);
    void fillCloud(Cloud& cloud, const std::string& digit);

    void createDirIsNeeded(std::string path);
    void copyCI(std::string from, std::string to);
  };
}
#endif /* OBJECTCONVERTER_H_ */
