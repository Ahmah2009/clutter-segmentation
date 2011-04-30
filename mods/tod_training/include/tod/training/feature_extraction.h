/*
 * features.h
 *
 *  Created on: Nov 4, 2010
 *      Author: ethan
 */

#ifndef FEATURES_H_TOD_
#define FEATURES_H_TOD_

#include "tod/training/feature_params.h"
#include "tod/training/stats.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tod/core/Features2d.h>

#include <map>
#include "rbrief/RBrief.h"
#include "rbrief/StopWatch.h"

namespace tod {
/** \brief interface to fill out a Features2d object with keypoints and descriptors
 */
class FeatureExtractor
{
public:
  virtual ~FeatureExtractor()
  {
  }
  virtual void detectAndExtract(Features2d& features) const = 0;
  static FeatureExtractor* create(FeatureExtractionParams params, detector_stats & stats);
  static cv::FeatureDetector* createDetector(const std::string& detectorType, float threshold, detector_stats & stats);
  
  // for backwards compatibility
  static FeatureExtractor* create(FeatureExtractionParams params);
  static cv::FeatureDetector* createDetector(const std::string& detectorType, float threshold);
};

/** \brief Given feature detector and extractor, compute features and descriptors simultaneously
 *   at different scales
 *   \code
 #Pseudo code for multi scale features + descriptors
 N_OCTAVES = 3
 scale = 1.0
 eps = scale / pow(2, N_OCTAVES)
 do:
 detect(img,keypoints)
 extract(img,descriptions)
 for x in keypoints:
 x.scale = scale
 x.pt = x.pt * ( 1 / scale) #rescale the point so that its relative to the original image
 resize( img, img.size/2 )
 scale = scale / 2
 while(scale > eps)
 *   \endcode
 */
class MultiscaleExtractor : public FeatureExtractor
{
public:
  MultiscaleExtractor(const cv::Ptr<cv::FeatureDetector>& d, const cv::Ptr<cv::DescriptorExtractor>& e, int n_octaves);
  MultiscaleExtractor()
  {
  }
  template<typename Detector, typename Extractor>
    MultiscaleExtractor(const Detector& d, const Extractor& e, int n_octaves) :
      detector_(new Detector(d)), extractor_(new Extractor(e)), n_octaves_(n_octaves)
    {

    }

  virtual void detectAndExtract(Features2d& features) const;
private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  int n_octaves_;
};

class OrbExtractor : public FeatureExtractor
{
public:
  enum DetectorType
  {
    ANMSFast, HarrisFast, Fast
  };
  OrbExtractor(float scale_factor, int n_levels, int n_desired_features,
                   rbrief::RBriefDescriptorExtractor::PatchSize ps = rbrief::RBriefDescriptorExtractor::PATCH_LEARNED,
                          bool steering = true, DetectorType detector = ANMSFast);
  void detectAndExtract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                        const cv::Mat& mask = cv::Mat()) const;
  virtual void detectAndExtract(Features2d& features) const;
  void debugDisplay() const;
  void setGetPatches(bool getPatches)
  {
    get_patches_ = getPatches;
  }
  void getPatches(cv::Mat& patches) const;
  void getPatches(std::vector<cv::Mat>& patches) const;
  const rbrief::StopWatch& getStopWatch() const
  {
    return watch_;
  }
private:
  mutable rbrief::StopWatch watch_;
  float scale_factor_;
  int n_levels_;
  int n_desired_features_;
  mutable std::vector<cv::Mat> working_mats_;
  mutable std::vector<cv::Mat> working_masks_;
  mutable std::vector<std::vector<cv::KeyPoint> > working_kpts_;
  mutable std::vector<cv::Mat> working_descriptors_;
  mutable std::vector<cv::Ptr<rbrief::RBriefDescriptorExtractor> > rbriefs_;
  std::vector<cv::Ptr<cv::FeatureDetector> > feature_detectors_;
  mutable std::vector<cv::Mat> patches_;
  bool get_patches_;
};

class SequentialExtractor : public FeatureExtractor
{
public:
  SequentialExtractor(const cv::Ptr<cv::FeatureDetector>& d, const cv::Ptr<cv::DescriptorExtractor>& e);
  SequentialExtractor()
  {
  }

  template<typename Detector, typename Extractor>
    SequentialExtractor(const Detector& d, const Extractor& e) :
      detector_(new Detector(d)), extractor_(new Extractor(e))
    {

    }
  virtual ~SequentialExtractor()
  {
  }

  virtual void detectAndExtract(Features2d& features) const;

private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

};

/** \brief a nop extractor, for when you don't want to extract or detect.
 *
 */
class NOPExtractor : public FeatureExtractor
{
public:
  virtual ~NOPExtractor()
  {
  }
  virtual void detectAndExtract(Features2d& features) const
  {
  }
};

/** \brief a file based extractor, for when you have a file of detected features
 *
 */
class FileExtractor : public FeatureExtractor
{
public:
  FileExtractor(const std::string& f2dname);
  virtual ~FileExtractor()
  {
  }
  virtual void detectAndExtract(Features2d& features) const;
private:
  std::string f2dname_;
};

/** convert from a vector of 'keypoints' with N-d data to 2d  xy points
 */
void KeyPointsToPoints(const KeypointVector& keypts, std::vector<cv::Point2f>& pts);
void PointsToKeyPoints(const std::vector<cv::Point2f>& keypts, KeypointVector& pts);

}/*namespace tod*/

#endif /* FEATURES_H_ */
