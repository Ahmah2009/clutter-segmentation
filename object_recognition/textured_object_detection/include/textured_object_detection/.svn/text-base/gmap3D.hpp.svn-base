#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "posest/pnp_ransac.h"

class GMap3D //: public GMap
{
public:
  GMap3D(const cv::Mat &_rvec, const cv::Mat &_tvec, const cv::Mat &_cameraMatirx, const cv::Mat &_distCoeffs);
  virtual void mapPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dst) const;

protected:
  cv::Mat rvec, tvec;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
};
