/*
 * detectors.h
 *
 *  Created on: Feb 11, 2011
 *      Author: erublee
 */

#ifndef DETECTORS_H_
#define DETECTORS_H_
#include <opencv2/features2d/features2d.hpp>
#include "rbrief/StopWatch.h"
#include "rbrief/RBrief.h"
namespace rbrief
{
class RBriefFastAdjuster : public cv::AdjusterAdapter
{
public:
  RBriefFastAdjuster(int initial_thesh = 80, bool fast_non_max = true, int low = 15, int high = 255);
  /** too few features were detected so, adjust the detector params accordingly
   * \param min the minimum number of desired features
   * \param n_detected the number previously detected
   */
  virtual void tooFew(int min, int n_detected);
  /** too many features were detected so, adjust the detector params accordingly
   * \param max the maximum number of desired features
   * \param n_detected the number previously detected
   */
  virtual void tooMany(int max, int n_detected);
  /** are params maxed out or still valid?
   * \return false if the parameters can't be adjusted any more
   */
  virtual bool good() const;
  /** are params maxed out or still valid? 
   * \return false if the parameters can't be adjusted any more 
   */ 
  virtual cv::Ptr<AdjusterAdapter> clone() const; 
protected:
  void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const;
  int thresh_;
  bool non_max_;
  int low_, high_;
};

struct HarrisResponse
{
  explicit HarrisResponse(const cv::Mat& image, double k = 0.04);
  void operator()(std::vector<cv::KeyPoint>& kpts) const;
private:
  /** The cached image to analyze
   */
  cv::Mat image;

  /** The k factor in the Harris corner detection
   */
  double k;

  /** The offset in X to compute the differences
   */
  std::vector<int> dX_offsets_;

  /** The offset in Y to compute the differences
   */
  std::vector<int> dY_offsets_;
};

class ANMSFastDetector : public cv::FeatureDetector
{
public:
  ANMSFastDetector() :
    desired_n_features_(100), threshhold_(20)
  {
  }
  ANMSFastDetector(size_t desired_n_features, float threshold = 20);
  mutable StopWatch watch;
protected:
  virtual void
               detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
  size_t desired_n_features_;
  float threshhold_;
};

class SimpleFASTHarris : public cv::FeatureDetector
{
public:
  SimpleFASTHarris() :
    desired_n_features_(100), use_harris_(true)
  {
  }
  SimpleFASTHarris(size_t desired_n_features, bool use_harris) :
    desired_n_features_(desired_n_features), use_harris_(use_harris)
  {
  }
  mutable StopWatch watch;
protected:
  virtual void
               detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
  size_t desired_n_features_;
  bool use_harris_;
};

class SimpleSURF : public cv::FeatureDetector
{
public:
  SimpleSURF()
  {
  }
  mutable StopWatch watch;
protected:
  virtual void
               detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
};

class MultiScaleDetectExtract
{
public:
  enum DetectorType
  {
    ANMSFast, HarrisFast, Fast
  };
  MultiScaleDetectExtract(float scale_factor, int n_levels, int n_desired_features,
                          RBriefDescriptorExtractor::PatchSize ps = RBriefDescriptorExtractor::PATCH_LEARNED,
                          bool steering = true, DetectorType detector = ANMSFast, int zoom_level_ = 0);
  void detectAndExtract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                        const cv::Mat& mask = cv::Mat());
  void debugDisplay() const;
  void setGetPatches(bool getPatches)
  {
    get_patches_ = getPatches;
  }
  void getPatches(cv::Mat& patches) const;
  void getPatches(std::vector<cv::Mat>& patches) const;
  const StopWatch& getStopWatch() const
  {
    return watch_;
  }
private:
  mutable StopWatch watch_;
  float scale_factor_;
  int n_levels_;
  int n_desired_features_;
  std::vector<cv::Mat> working_mats_;
  std::vector<cv::Mat> working_masks_;
  std::vector<std::vector<cv::KeyPoint> > working_kpts_;
  std::vector<cv::Mat> working_descriptors_;
  std::vector<cv::Ptr<RBriefDescriptorExtractor> > rbriefs_;
  std::vector<cv::Ptr<cv::FeatureDetector> > feature_detectors_;
  std::vector<cv::Mat> patches_;
  bool get_patches_;
  int zoom_level_;
};

}

#endif /* DETECTORS_H_ */
