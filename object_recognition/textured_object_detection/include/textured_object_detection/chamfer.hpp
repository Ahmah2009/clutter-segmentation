#if !defined(_CHAMFER_HPP)
#define _CHAMFER_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chamfer_matching/chamfer_matching.h>
#include "textured_object_detection/gmap3D.hpp"

class GMatchCM //: public GMatch
{
public:
  class Params
  {
  public:
    Params(double _canny_thresh1 = 80.0, double _canny_thresh2 = 120.0) :
      canny_thresh1(_canny_thresh1), canny_thresh2(_canny_thresh2)
    {
    }

    double canny_thresh1, canny_thresh2;
  };

  GMatchCM(GMatchCM::Params _edges_params = GMatchCM::Params());

  ~GMatchCM()
  {
  }

  //virtual void setTemplateImage(const Mat& src);
  virtual void setTemplateEdgels(const vector<cv::Point3f>& _edgels);
  virtual void setTestImage(const cv::Mat& src);
  virtual float calcMatchCost(const GMap3D& gmap) const;
  virtual void showMatch(const GMap3D& gmap) const;

protected:
  static void cropImage(cv::Mat &src, int width);
  void mapEdgels(const GMap3D &gmap, cv::Mat &dstImage, cv::Point &offset) const;

  //vector<vector<Point> > contours;
  vector<cv::Point3f> edgels;
  cv::Mat test_image;
  Params edges_params;
};

#endif
