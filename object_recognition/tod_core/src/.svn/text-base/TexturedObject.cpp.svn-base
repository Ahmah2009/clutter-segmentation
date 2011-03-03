/*
 * TexturedObject.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#include <tod/core/TexturedObject.h>
#include <opencv2/core/core_c.h>
namespace tod
{

TexturedObject::TexturedObject():id(-1), stddev(-1.0)
{

}

TexturedObject::~TexturedObject()
{
}

std::vector<cv::Mat> TexturedObject::getDescriptors() const
{
  std::vector<cv::Mat> ds;
  ds.reserve(observations.size());
  for (size_t i = 0; i < observations.size(); i++)
  {
    ds.push_back(observations[i].features().descriptors);
  }
  return ds;
}

//serialization
void TexturedObject::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs, "TexturedObject", 0);

  fs << "{";
  fs << "id" << id << "name" << name;
  fs << "observations" << "[";
  for (size_t i = 0; i < observations.size(); i++)
  {
    observations[i].write(fs);
  }
  fs << "]";

  fs << "}";

}
void TexturedObject::read(const cv::FileNode& fn)
{
  id = (int)fn["id"];
  name = (std::string)fn["name"];
  cv::FileNode os = fn["observations"];
  CV_Assert(os.type() == cv::FileNode::SEQ)
    ;
  observations.resize(os.size());
  for (size_t i = 0; i < os.size(); i++)
  {
    observations[i].read(os[i]);
  }
}

}
