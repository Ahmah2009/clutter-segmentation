/*
 * fitting.h
 *
 *  Created on: Nov 4, 2010
 *      Author: ethan
 */

#ifndef FITTING_H_
#define FITTING_H_

#include "tod/core/Features3d.h"
#include "tod/core/TexturedObject.h"
#include <opencv2/core/core.hpp>
namespace tod
{

template<typename ObservationT, typename PoseT = PoseRT>
  class PoseEstimator
  {
  public:
    typedef PoseT Pose;
    typedef ObservationT Observation;
    virtual ~PoseEstimator()
    {
    }
    virtual Pose estimatePose(const Observation& observation) const = 0;

  };
typedef PoseEstimator<cv::Mat, PoseRT> ImageBasedPE;

class CheckerboardPoseEstimator : public ImageBasedPE
{
public:
  CheckerboardPoseEstimator(const cv::Size& chess_size, double square_size, const Camera& camera);

  virtual Pose estimatePose(const Observation& observation) const;

  static std::vector<cv::Point3f> CalcChessboardCorners(cv::Size chess_size, float square_size);
  static float ScaleImage(cv::Mat & image, float desired_width);
  static bool AccurateChesscorners(const cv::Mat& image_, cv::Size chess_size, std::vector<cv::Point2f>& corners,
                                   float scale_factor = 1, const cv::Mat low_res = cv::Mat());
  static cv::Mat_<double> ScaleK(const cv::Mat& K, float scale_factor)
  {
    cv::Mat_<double> sK = K / scale_factor;
    sK(2, 2) = 1;
    return sK;
  }
private:
  cv::Size chess_size_;
  float square_size_;
  Camera camera_;
  cv::Mat K_;
  std::vector<cv::Point3f> boardPoints_;
};

class Fiducial : public Serializable{
public:
  typedef std::vector<cv::Point3f> Points;
  enum Type {
    CHECKER_BOARD = 0,
    DOTS
  };
  Fiducial():type_(CHECKER_BOARD){}
  Fiducial(Type ftype, const vector<Points>& patterns);
  Fiducial(const std::vector<cv::Size>& corner_counts, const std::vector<float>& spacings, const std::vector<cv::Point3f>& offsets,Type ftype = CHECKER_BOARD);
  void detect(const cv::Mat& test_img, std::vector<vector<cv::Point2f> >& observations, vector<bool>& found) const;
  void draw(cv::Mat& drawimage, const std::vector<vector<cv::Point2f> >& observations, const vector<bool>& found ) const;
  const std::vector<Points>& getTemplates() const;
  const cv::Size& getCounts(int idx) const{
    return corner_counts_[idx];
  }
  virtual void write(cv::FileStorage& fs) const;
  virtual void read(const cv::FileNode& fn);

private:
  Type type_;
  std::vector<Points> templates_;
  std::vector<cv::Size> corner_counts_;
};

class FiducialPoseEstimator : public ImageBasedPE
{
public:
  FiducialPoseEstimator(const Fiducial& fudicial, const Camera& camera,bool verbose = true);
  virtual Pose estimatePose(const Observation& observation) const;
private:
  Fiducial fiducial_;
  Camera camera_;
  bool verbose_;
};


class KnownPoseEstimator : public ImageBasedPE
{
public:
  KnownPoseEstimator(const std::string& pose_file);
  KnownPoseEstimator(Pose& pose) :
    pose_(pose)
  {
  }
  virtual Pose estimatePose(const Observation& observation) const
  {
    return pose_;
  }
private:
  Pose pose_;
};

void write(cv::FileStorage fs, const std::string name, const PoseRT& pose);
void read(cv::FileNode node, PoseRT& pose);



/** \brief given a line in point intercept form, generate two points on this
 * line
 *
 * This function is useful for drawing epipolar lines on an image
 * \param l a line in form (a,b,c) ax + by + c = 0
 * \param p1 output point
 * \param p2 output point
 * \param sz the size of the output image ( used for making the points span)
 */
void ComputeLinePoints(const cv::Vec3f& l, cv::Point2f & p1, cv::Point2f& p2, cv::Size sz);

void DrawEpilines(const std::vector<cv::Vec3f>& lines, cv::Mat& outputimage, cv::Scalar color);

void EpiliPolarDrawer(cv::Mat image1, cv::Mat image2, cv::Mat F);

void PoseDrawer(cv::Mat& drawImage, const cv::Mat& K, const PoseRT& pose);
}

#endif /* FITTING_H_ */
