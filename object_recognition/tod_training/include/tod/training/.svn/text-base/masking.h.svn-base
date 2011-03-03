/*
 * masking.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#ifndef TOD_MASKING_H_
#define TOD_MASKING_H_

#include <tod/core/Features2d.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace tod
{
/** \brief interface to mask off the area of interest for a Features2d object
 */
class Masker
{
public:
  virtual ~Masker()
  {
  }
  virtual void mask(Features2d& features) const = 0;
};

/** \brief a NOP masker, that doesn't do anything.
 */
class NonMasker : public Masker
{
public:
  virtual void mask(Features2d& features) const
  {
  }
};

/** \brief Given an image file or filename mask using a black and white image
 */
class FileMasker : public Masker
{
public:
  FileMasker(const std::string& mask_file);
  FileMasker(const cv::Mat mask_image);
  virtual void mask(Features2d& features) const
  {
    if (!mask_.empty())
      mask_.copyTo(features.mask);
  }
private:
  cv::Mat mask_;
};

/** \brief Use a green screen to do the masking
 */
class GreenScreenMasker : public Masker
{
public:
  virtual void mask(Features2d& features) const;
};

/** \brief Use a vertex defined countour to do masking
 */
class ContourMasker : public Masker
{
public:
  virtual void mask(Features2d& features) const;
};

/** \brief use a homography and 3d box to do the masking...
 * \ingroup features
 */
class Select3dObjecMasker : public Masker
{

public:
  Select3dObjecMasker(const Camera& camerar);
  bool guiSelectBox(const Features2d& features);
  void setBox(const std::vector<cv::Point3f>& box);
  const std::vector<cv::Point3f>& getBox() const;
  virtual void mask(Features2d& features) const;
private:
  Camera camera_;
  std::vector<cv::Point3f> box_;
};

class Convex3dHullMasker: public Masker
{
public:
  virtual void mask(Features2d& features) const;
};


class PointCloudMasker:public Masker
{
public:
  typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
  void compute3dPointsOfInterest(const cloud_t& cloud,const PoseRT& pose, const Camera& camera, const cv::Vec3f& bounding_range);
  virtual void mask(Features2d& features) const;
private:
  boost::shared_ptr<cloud_t> cloud_;
};

cv::Mat cloudMask(const PointCloudMasker::cloud_t& cloud, const PoseRT& pose, const Camera& camera);


}
#endif /* MASKING_H_ */
