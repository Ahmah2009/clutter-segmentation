/*
 * TODTrainer.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#ifndef TODTRAINER_H_
#define TODTRAINER_H_

#include <tod/core/TexturedObject.h>
#include <tod/training/feature_extraction.h>
#include <string>
namespace tod
{
struct TODParams
{
  std::string directory;
  std::string camera_file;
  std::string feature_extraction_params;
  std::vector<std::string> image_names;
  std::vector<std::string> pcd_names;
  Camera openCamera() const;
  cv::Mat openImage(int idx) const;
  void openPcl(int idx, Cloud& cloud)const;
  FeatureExtractionParams openFeatureExtractionParams() const;
};
class TODTrainer
{
public:
  TODTrainer(const TODParams& params);
  cv::Ptr<TexturedObject> train();
private:
  TODParams params_;
};

}
#endif /* TODTRAINER_H_ */
