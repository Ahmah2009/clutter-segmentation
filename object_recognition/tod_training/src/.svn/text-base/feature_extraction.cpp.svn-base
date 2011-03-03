/*
 * feature_extraction.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */
#include <tod/training/feature_extraction.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <rbrief/RBrief.h>
#include <rbrief/detectors.h>
using std::cout;
using namespace cv;
namespace tod
{

void FeatureExtractionParams::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs, "FeatureExtractionParams", 0);
  fs << "{";
  fs << "detector_type" << detector_type << "extractor_type" << extractor_type << "descriptor_type" << descriptor_type;
  fs << "detector_params" << "{";
  std::map<std::string, double>::const_iterator it = detector_params.begin();
  for (; it != detector_params.end(); ++it)
  {
    fs << it->first << it->second;
  }
  fs << "}";
  fs << "extractor_params" << "{";
  it = extractor_params.begin();
  for (; it != extractor_params.end(); ++it)
  {
    fs << it->first << it->second;
  }
  fs << "}";
  fs << "}";

}
void FeatureExtractionParams::read(const cv::FileNode& fn)
{
  detector_type = (string)fn["detector_type"];
  extractor_type = (string)fn["extractor_type"];
  descriptor_type = (string)fn["descriptor_type"];

  cv::FileNode params = fn["detector_params"];
  CV_Assert(params.type() == cv::FileNode::MAP)
    ;
  cv::FileNodeIterator it = params.begin();
  for (; it != params.end(); ++it)
  {
    detector_params[(*it).name()] = (double)(*it);
    //cout << "read:" << (*it).name() << " = " << detector_params[(*it).name()] << endl;
  }
  params = fn["extractor_params"];
  CV_Assert(params.type() == cv::FileNode::MAP)
    ;

  it = params.begin();
  for (; it != params.end(); ++it)
  {
    extractor_params[(*it).name()] = (double)(*it);
    //cout << "read:" << (*it).name() << " = " << extractor_params[(*it).name()] << endl;
  }

}

FeatureExtractionParams FeatureExtractionParams::CreateSampleParams()
{
  FeatureExtractionParams params;
  params.descriptor_type = "BRIEF";
  params.detector_type = "FAST";
  params.extractor_type = "multi-scale";
  params.extractor_params["octaves"] = 3;
  params.detector_params["min_features"] = 500;
  params.detector_params["max_features"] = 700;
  params.detector_params["threshold"] = 200;
  return params;
}
const std::string FeatureExtractionParams::YAML_NODE_NAME = "feature_extraction_params";
FeatureDetector* FeatureExtractor::createDetector(const string& detectorType, float threshold)
{
  FeatureDetector* fd = 0;
  if (!detectorType.compare("FAST"))
  {
    fd  = new FastFeatureDetector(threshold/*threshold*/, true/*nonmax_suppression*/);
  }
  else if (!detectorType.compare("STAR"))
  {
    fd  = new StarFeatureDetector(16/*max_size*/, (int)threshold/*response_threshold*/,
                                  10/*line_threshold_projected*/, 8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/);
  }
  else if (!detectorType.compare("SIFT"))
  {
    fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  }
  else if (!detectorType.compare("SURF"))
  {
    fd = new SurfFeatureDetector(threshold/*hessian_threshold*/, 5/*octaves*/, 4/*octave_layers*/);
  }
  else if (!detectorType.compare("MSER"))
  {
    fd = new MserFeatureDetector(5/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.25f/*max_variation*/,
                                 0.2/*min_diversity*/, 200/*max_evolution*/,
                                 threshold/*area_threshold*/, 0.003/*min_margin*/, 5/*edge_blur_size*/);
  }
  else if (!detectorType.compare("GFTT"))
  {
    fd = new GoodFeaturesToTrackDetector(1000/*maxCorners*/, threshold/*qualityLevel*/,
                                         1./*minDistance*/, 3/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/);
  }
  else    assert(0);
  return fd;
}

FeatureExtractor* FeatureExtractor::create(FeatureExtractionParams params)
{
  FeatureExtractor* fe = 0;
  int min_features = params.detector_params["min_features"];
  int max_features = params.detector_params["max_features"];
  cv::Ptr<FeatureDetector> detector;
  if (params.detector_type == "DynamicFAST")
  {
    detector = new DynamicAdaptedFeatureDetector(new rbrief::RBriefFastAdjuster(80,true,10,255), min_features, max_features, 200);
  }
  else if (params.detector_type == "DynamicSTAR")
  {
    detector = new DynamicAdaptedFeatureDetector(new StarAdjuster(), min_features, max_features, 200);
  }
  else if (params.detector_type == "DynamicSURF")
    detector = new DynamicAdaptedFeatureDetector(new SurfAdjuster(), min_features, max_features, 200);
  else if (params.detector_params.end() != params.detector_params.find("threshold"))
    detector = FeatureExtractor::createDetector(params.detector_type, params.detector_params["threshold"]);

  else detector = FeatureDetector::create(params.detector_type);

  // Define the extractor
  cv::Ptr<DescriptorExtractor> extractor;
    if (params.descriptor_type == "rBRIEF")
    {
      rbrief::RBriefDescriptorExtractor * rbrief_extractor = new rbrief::RBriefDescriptorExtractor();
      // Set the patch size
      //switch ((int) params.extractor_params["patch_size"])
      switch (32)
      {
        case 16:
        rbrief_extractor->setPatchSize(rbrief::RBriefDescriptorExtractor::PATCH_16);
        break;
        case 32:
        rbrief_extractor->setPatchSize(rbrief::RBriefDescriptorExtractor::PATCH_32);
        break;
        case 48:
        rbrief_extractor->setPatchSize(rbrief::RBriefDescriptorExtractor::PATCH_48);
        break;
      }
      // Set the steerer
      //int kernel_size = params.extractor_params["half_kernel_size"];
      int half_kernel_size = 15;
      Ptr<rbrief::KeypointSteerer> steerer(new rbrief::IntensityCentroid(half_kernel_size));
      rbrief_extractor->setSteerer(steerer);
      extractor = rbrief_extractor;
    }
    else
    extractor = DescriptorExtractor::create(params.descriptor_type);
  if(extractor.empty())
    throw std::runtime_error("bad extractor");
  if (params.extractor_type == "multi-scale")
  {
    fe = new MultiscaleExtractor(detector, extractor, params.extractor_params["octaves"]);
  }
  else if (params.extractor_type == "sequential")
  {
    fe = new SequentialExtractor(detector, extractor);
  }
  return fe;

}
MultiscaleExtractor::MultiscaleExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e, int n_octaves) :
  detector_(d), extractor_(e), n_octaves_(n_octaves)
{

}

void MultiscaleExtractor::detectAndExtract(Features2d& features) const
{
  int octaves = n_octaves_;
  Mat image = features.image.clone();

  float scale_factor = sqrt(2);
  Mat mask = features.mask.empty() ? Mat() : features.mask.clone();

  float scale_x = 1.0f;
  float scale_y = 1.0f;
  for (int i = 0; i < octaves; i++)
  {
    vector<KeyPoint> kpts;
    Mat descriptors;
    detector_->detect(image, kpts, mask);
    extractor_->compute(image, kpts, descriptors);

    for (size_t j = 0; j < kpts.size(); j++)
    {
      kpts[j].pt.x *= scale_x;
      kpts[j].pt.y *= scale_y;
    }

    if (i < octaves - 1)
    {
      scale_x = features.image.cols / (image.cols / scale_factor);
      scale_y = features.image.rows / (image.rows / scale_factor);

      Size n_size(image.cols / scale_factor, image.rows / scale_factor);
      resize(features.image, image, n_size);
      if (!features.mask.empty())
        resize(features.mask, mask, n_size);
    }
    features.keypoints.insert(features.keypoints.end(), kpts.begin(), kpts.end());
    Mat n_desc(features.descriptors.rows + descriptors.rows, extractor_->descriptorSize(), extractor_->descriptorType());
    Mat top_desc(n_desc.rowRange(Range(0, features.descriptors.rows)));
    Mat bottom_desc(n_desc.rowRange(Range(features.descriptors.rows, features.descriptors.rows + descriptors.rows)));

    features.descriptors.copyTo(top_desc);
    descriptors.copyTo(bottom_desc);
    features.descriptors = n_desc;
#if 0
    imshow("octave", image);
    imshow("scaled mask", mask);
    waitKey(0);
#endif
  }
}
SequentialExtractor::SequentialExtractor(const cv::Ptr<cv::FeatureDetector>& d,
                                         const cv::Ptr<cv::DescriptorExtractor>& e) :
  detector_(d), extractor_(e)
{

}
void SequentialExtractor::detectAndExtract(Features2d& features) const
{
  detector_->detect(features.image, features.keypoints, features.mask);
  extractor_->compute(features.image, features.keypoints, features.descriptors);
}

FileExtractor::FileExtractor(const std::string& f2dname) :
  f2dname_(f2dname)
{

}
void FileExtractor::detectAndExtract(Features2d& features) const
{
  cv::FileStorage fs(f2dname_, cv::FileStorage::READ);
  cv::read(fs["keypoints"], features.keypoints);
  //    fs["keypoints"] >> features.keypoints;
  fs["descriptors"] >> features.descriptors;
}
void KeyPointsToPoints(const KeypointVector& keypts, std::vector<cv::Point2f>& pts)
{
  pts.clear();
  pts.reserve(keypts.size());
  for (size_t i = 0; i < keypts.size(); i++)
  {
    pts.push_back(keypts[i].pt);
  }
}

void PointsToKeyPoints(const std::vector<cv::Point2f>& pts, KeypointVector& kpts)
{
  kpts.clear();
  kpts.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); i++)
  {
    kpts.push_back(KeyPoint(pts[i], 6.0));
  }
}

}
