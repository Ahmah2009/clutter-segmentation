/*
 * features.h
 *
 *  Created on: Nov 4, 2010
 *      Author: ethan
 */

#ifndef FEATURES_H_TOD_
#define FEATURES_H_TOD_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tod/core/Features2d.h>

#include <map>

namespace tod
{
struct FeatureExtractionParams : public Serializable
{

  std::string detector_type;
  std::string descriptor_type;
  std::string extractor_type;
  std::map<string, double> detector_params;
  std::map<string, double> extractor_params;
  //serialization
  virtual void write(cv::FileStorage& fs) const;
  virtual void read(const cv::FileNode& fn);

  static FeatureExtractionParams CreateSampleParams();
  static const std::string YAML_NODE_NAME;
};
/** \brief interface to fill out a Features2d object with keypoints and descriptors
 */
class FeatureExtractor
{
public:
  virtual ~FeatureExtractor()
  {
  }
  virtual void detectAndExtract(Features2d& features) const = 0;
  static FeatureExtractor* create(FeatureExtractionParams params);
  static cv::FeatureDetector* createDetector(const string& detectorType, float threshold);
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
