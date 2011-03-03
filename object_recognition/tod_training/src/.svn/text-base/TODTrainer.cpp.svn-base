/*
 * TODTrainer.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#include <tod/training/TODTrainer.h>
#include <opencv2/highgui/highgui.hpp>

namespace tod
{

TODTrainer::TODTrainer(const TODParams& params) :
  params_(params)
{

}
cv::Mat TODParams::openImage(int idx) const
{
  //TODO allow for color image reading
  cv::Mat img = cv::imread(directory + "/" + image_names[idx],CV_LOAD_IMAGE_GRAYSCALE);
  CV_Assert(!img.empty())
    ;
  return img;
}
void TODParams::openPcl(int idx, Cloud& cloud) const
{
  cv::FileStorage fs(directory + "/" + pcd_names[idx], cv::FileStorage::READ);
  CV_Assert(fs.isOpened())
    ;
  cv::FileNode cn = fs["cloud"];
  cloud.clear();
  cn >> cloud;

}
FeatureExtractionParams  TODParams::openFeatureExtractionParams() const{
  cv::FileStorage fs(directory + "/" + camera_file, cv::FileStorage::READ);
   FeatureExtractionParams fp;
   fp.read(fs["feature_extraction_params"]);
   return fp;
}
Camera TODParams::openCamera() const{
  cv::FileStorage fs(directory + "/" + camera_file, cv::FileStorage::READ);
  Camera c;
  c.read(fs["camera"]);
  return c;
}
cv::Ptr<TexturedObject> TODTrainer::train()
{
  cv::Ptr<TexturedObject> obj(new TexturedObject());
  Camera camera(params_.openCamera());
  FeatureExtractionParams params(params_.openFeatureExtractionParams());
  CV_Assert(params_.pcd_names.size() == 0 || params_.pcd_names.size() == params_.image_names.size());
  for (size_t i = 0; i < params_.image_names.size(); i++)
  {
    Features2d f2d(camera,params_.openImage(i));
    f2d.image_name = params_.directory + "/" + params_.image_names[i];
    Cloud cloud;
    if(params_.pcd_names.size()){
      params_.openPcl(i,cloud);
    }
    Features3d f3d(f2d,cloud);
  }
  return NULL;
}

}
