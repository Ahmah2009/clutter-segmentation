/*
 * pose.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#include <tod/core/PoseRT.h>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
namespace tod
{

const std::string PoseRT::YAML_NODE_NAME = "pose";
PoseRT::PoseRT() :
  rvec(cv::Mat_<float>::zeros(3, 1)), tvec(cv::Mat_<float>::zeros(3, 1)),estimated(false)
{

}

PoseRT::PoseRT(const cv::Mat rvec, const cv::Mat& tvec) :
  rvec(rvec.clone()), tvec(tvec.clone()),estimated(true) //prevent modification by reference!
{

}
//serialization
void PoseRT::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs,"PoseRT",0);
  fs << "{" << "rvec" << rvec << "tvec" << tvec << "estimated" << (int)estimated << "}";
}
void PoseRT::read(const cv::FileNode& fn)
{
  fn["rvec"] >> rvec;
  fn["tvec"] >> tvec;
  estimated = (int)fn["estimated"];
}

cv::Vec4f PoseRT::toPlanarCoefficients() const{
  cv::Vec4f coefficients;
  cv::Mat R;
  cv::Rodrigues(rvec,R);
  R.clone().convertTo(R,CV_32F);
  cv::Mat t;
  tvec.convertTo(t,CV_32F);
  cv::Mat n = (cv::Mat_<float>(3,1) << 0,0,1);
  {
    cv::Mat Z = (cv::Mat_<float>(3,1) << 0,0,1);
    n = R*Z; // n is unit vector normal to plane
  }
  float d = n.dot(-t); //plane given by n.(r - r_0) = 0, where r_0 is tvec;
  coefficients[0] = n.at<float>(0);
  coefficients[1] = n.at<float>(1);
  coefficients[2] = n.at<float>(2);
  coefficients[3] = d;
  return coefficients;
}

void  PoseRT::enforceForm(){
  CV_Assert(tvec.size().area() == 3 && (rvec.size().area() == 3 || rvec.size().area() == 9));
  if(rvec.size().area()== 9){
    cv::Rodrigues(rvec.clone(),rvec);
  }
}
}
