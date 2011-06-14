#include <rbrief/detectors.h>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>
#include <iostream>
#include <boost/foreach.hpp>
#include <rbrief/testing.h>
using namespace cv;
namespace rbrief
{

RBriefFastAdjuster::RBriefFastAdjuster(int initial_thesh, bool fast_non_max, int low, int high) :
  thresh_(initial_thesh), non_max_(fast_non_max), low_(low), high_(high)
{
}
void RBriefFastAdjuster::tooFew(int min, int n_detected)
{
  thresh_ -= 1/* + (min - n_detected) / 10*/;

}
void RBriefFastAdjuster::tooMany(int max, int n_detected)
{
  thresh_ += 1/* + (n_detected - max) / 10*/;
}
bool RBriefFastAdjuster::good() const
{
  return thresh_ > low_ && thresh_ < high_;
}
cv::Ptr<AdjusterAdapter> RBriefFastAdjuster::clone() const 
{ 
    cv::Ptr<AdjusterAdapter> copy = new RBriefFastAdjuster(this->thresh_, this->non_max_, this->low_, this->high_); 
    return copy; 
} 

void RBriefFastAdjuster::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) const
{
  FastFeatureDetector(thresh_, non_max_).detect(image, keypoints, mask);
}
template<typename PatchType, typename SumType>
  inline float harris(const cv::Mat& patch, float k, const std::vector<int> &dX_offsets,
                      const std::vector<int> &dY_offsets)
  {
    float a = 0, b = 0, c = 0;

    static cv::Mat_<SumType> dX(9, 7), dY(7, 9);
    SumType * dX_data = reinterpret_cast<SumType*> (dX.data), *dY_data = reinterpret_cast<SumType*> (dY.data);
    SumType * dX_data_end = dX_data + 9 * 7;
    PatchType * patch_data = reinterpret_cast<PatchType*> (patch.data);
    int two_row_offset = 2 * patch.step1();
    std::vector<int>::const_iterator dX_offset = dX_offsets.begin(), dY_offset = dY_offsets.begin();
    // Compute the differences
    for (; dX_data != dX_data_end; ++dX_data, ++dY_data, ++dX_offset, ++dY_offset)
    {
      *dX_data = (SumType)(*(patch_data + *dX_offset)) - (SumType)(*(patch_data + *dX_offset - 2));
      *dY_data = (SumType)(*(patch_data + *dY_offset)) - (SumType)(*(patch_data + *dY_offset - two_row_offset));
    }

    // Compute the Scharr result
    dX_data = reinterpret_cast<SumType*> (dX.data);
    dY_data = reinterpret_cast<SumType*> (dY.data);
    for (size_t v = 0; v <= 6; v++, dY_data += 2)
    {
      for (size_t u = 0; u <= 6; u++, ++dX_data, ++dY_data)
      {
        //float weight = 10 - std::sqrt((v - 4) * (v - 4) + (u - 4) * (u - 4));
        float weight = 1.0 / (9.0 * 9.0);
        float Ix = 3 * (*dX_data + *(dX_data + 14)) + 10 * (*(dX_data + 7));
        float Iy = 3 * (*dY_data + *(dY_data + 2)) + 10 * (*(dY_data + 1));

        Ix *= weight;
        Iy *= weight;
        a += Ix * Ix;
        b += Iy * Iy;
        c += Ix * Iy;
      }
    }

    return ((a * b - c * c) - (k * ((a + b) * (a + b))));
  }

HarrisResponse::HarrisResponse(const cv::Mat& image, double k) :
  image(image), k(k)
{
  // Compute the offsets for the Harris corners once and for all
  dX_offsets_.resize(7 * 9);
  dY_offsets_.resize(7 * 9);
  std::vector<int>::iterator dX_offsets = dX_offsets_.begin(), dY_offsets = dY_offsets_.begin();
  unsigned int image_step = image.step1();
  for (size_t y = 0; y <= 6 * image_step; y += image_step)
  {
    int dX_offset = y + 2, dY_offset = y + 2 * image_step;
    for (size_t x = 0; x <= 6; ++x)
    {
      *(dX_offsets++) = dX_offset++;
      *(dY_offsets++) = dY_offset++;
    }
    for (size_t x = 7; x <= 8; ++x)
      *(dY_offsets++) = dY_offset++;
  }

  for (size_t y = 7 * image_step; y <= 8 * image_step; y += image_step)
  {
    int dX_offset = y + 2;
    for (size_t x = 0; x <= 6; ++x)
      *(dX_offsets++) = dX_offset++;
  }
}

void HarrisResponse::operator()(std::vector<cv::KeyPoint>& kpts) const
{
  BOOST_FOREACH(cv::KeyPoint & kpt, kpts)
        {
          // make sure the keypoint and its neighborhood is fully in the image
          if ((kpt.pt.x - 4 < 0) || (kpt.pt.y - 4 < 0) || (kpt.pt.x + 4 >= image.cols) || (kpt.pt.y + 4 >= image.rows))
            kpt.response = 0;
          else
          {
            cv::Mat patch = image(cv::Rect(kpt.pt.x - 4, kpt.pt.y - 4, 9, 9));

            // Compute the response
#if 0
            cv::Mat_<float> Ix(9, 9), Iy(9, 9);

            cv::Scharr(patch, Ix, CV_32F, 1, 0);
            cv::Scharr(patch, Iy, CV_32F, 0, 1);
            Ix = Ix / (9.0 * 9.0);
            Iy = Iy / (9.0 * 9.0);
            float a = 0, b = 0, c = 0;
            for (unsigned int y = 1; y <= 7; ++y)
            {
              for (unsigned int x = 1; x <= 7; ++x)
              {
                a += Ix(y, x) * Ix(y, x);
                b += Iy(y, x) * Iy(y, x);
                c += Ix(y, x) * Iy(y, x);
              }
            }
            //[ a c ]
            //[ c b ]
            float response = (float)((a * b - c * c) - k * ((a + b) * (a + b)));
#endif
            kpt.response = harris<uchar, int> (patch, k, dX_offsets_, dY_offsets_);
#if 0
            std::cout << response - kpt.response << std::endl;
#endif
          }
        }
}
namespace
{
inline bool keypointResponseLess(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.response < rhs.response;
}

inline bool keypointResponseGreater(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.response > rhs.response;
}
inline bool keypointSizeGreater(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.size > rhs.size;
}
inline bool keypointDist(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
{
  return lhs.response < 0.99 * rhs.response ? lhs.pt.dot(rhs.pt) : INFINITY;
}

inline float minDist(const std::vector<cv::KeyPoint>& keypoints, const KeyPoint& kpt, int idx)
{
  float min = 5000;
  for (int i = idx - 1; i >= 0; --i)
  {
    const cv::KeyPoint& kpt_t = keypoints[i];
    if (kpt.response < 0.90f * kpt_t.response)
      break;
    float t_min = kpt.pt.dot(kpt_t.pt);
    min = std::min(t_min, min);
  }
  return min;
}
struct ANMS
{
  ANMS(std::vector<cv::KeyPoint>& keypoints) :
    kpts(keypoints), idx(1)
  {

  }
  inline void operator()(cv::KeyPoint& kpt)
  {
    kpt.size = minDist(kpts, kpt, idx);
    ++idx;
  }
  const std::vector<cv::KeyPoint>& kpts;
  int idx;
};
}

void SimpleSURF::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  SURF surf(4000, 4, 2, true); //extended
  surf(image, mask, keypoints);
}

void SimpleFASTHarris::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  FastFeatureDetector fd(20, true);
  fd.detect(image, keypoints, mask);
  size_t n_features = desired_n_features_;
  if (use_harris_)
    n_features *= 2;
  if (keypoints.size() > n_features)
  {
    std::nth_element(keypoints.begin(), keypoints.begin() + n_features, keypoints.end(), keypointResponseGreater);
    keypoints.resize(n_features);
    n_features /= 2;
  }

  if (use_harris_)
  {
    HarrisResponse h(image);
    h(keypoints);
    if (keypoints.size() > n_features)
    {
      std::nth_element(keypoints.begin(), keypoints.begin() + n_features, keypoints.end(), keypointResponseGreater);
      keypoints.resize(n_features);
    }
  }

}
ANMSFastDetector::ANMSFastDetector(size_t desired_n_features, float threshold) :
  desired_n_features_(desired_n_features), threshhold_(threshold)
{

}

void ANMSFastDetector::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{

  watch.start("detection");
  keypoints.clear();

  int rows = 4, cols = 4;
  cv::Rect roi_mother(0, 0, image.size().width / cols, image.size().height / rows);
  std::vector<cv::KeyPoint> kpts_all[16];
  //  omp_set_num_threads(8);
  //#pragma omp parallel for
  FastFeatureDetector fd(20, true);

  size_t total_n = 0;
  for (int ii = 0; ii < cols * rows; ii++)
  {
    cv::Mat mask_q;

    int j = ii % rows;
    int i = ii / cols;
    std::vector<cv::KeyPoint>& kpts = kpts_all[i * rows + j];
    Rect roi = roi_mother;
    roi.x = i * roi.width;
    roi.y = j * roi.height;
    if (!mask.empty())
    {
      mask_q = mask(roi);
    }
    if (!mask_q.empty() && cv::countNonZero(mask_q) < 10)
      continue;
    fd.detect(image(roi), kpts, mask_q);
    size_t top_f = desired_n_features_ / (rows * cols);
    if (kpts.size() > top_f * 3)
    {
      std::nth_element(kpts.begin(), kpts.begin() + top_f * 3, kpts.end(), keypointResponseGreater);
      kpts.resize(top_f * 3);
    }
    HarrisResponse h(image(roi));
    h(kpts);
    if (kpts.size() > top_f)
    {
      std::nth_element(kpts.begin(), kpts.begin() + top_f, kpts.end(), keypointResponseGreater);
      kpts.resize(top_f);
    }
    for (size_t k = 0; k < kpts.size(); k++)
    {
      kpts[k].pt.x += roi.x;
      kpts[k].pt.y += roi.y;
    }
    total_n += kpts.size();
  }

  keypoints.reserve(total_n);
  for (int i = 0; i < 16; i++)
  {
    keypoints.insert(keypoints.end(), kpts_all[i].begin(), kpts_all[i].end());
  }
  watch.stop("detection");

}

MultiScaleDetectExtract::MultiScaleDetectExtract(float scale_factor, int n_levels, int n_desired_features,
                                                 RBriefDescriptorExtractor::PatchSize ps, bool steering,
                                                 DetectorType detector, int mag_level) :
  scale_factor_(scale_factor), n_levels_(n_levels > 0 ? n_levels : 1), n_desired_features_(n_desired_features),
      working_mats_(n_levels_), working_masks_(n_levels_), working_kpts_(n_levels_), working_descriptors_(n_levels_),
      rbriefs_(n_levels_), get_patches_(false), zoom_level_(mag_level)
{
  for (size_t i = 0; i < rbriefs_.size(); i++)
  {
    rbriefs_[i] = new RBriefDescriptorExtractor();
    rbriefs_[i]->setPatchSize(ps);
    if (steering)
      rbriefs_[i]->setSteerer(new IntensityCentroid(15));
    else
      rbriefs_[i]->setSteerer(new UniformSteerer(0));
    switch (detector)
    {
      case ANMSFast:
        feature_detectors_.push_back(new ANMSFastDetector(n_desired_features_ * (1 / std::pow(scale_factor_, i))));
        break;
      case HarrisFast:
        feature_detectors_.push_back(
                                     new SimpleFASTHarris(n_desired_features_ * (1 / std::pow(scale_factor_, i)), false));
        break;
      case Fast:
        feature_detectors_.push_back(new cv::FastFeatureDetector(20));
        break;
    }
  }
}

void MultiScaleDetectExtract::getPatches(std::vector<cv::Mat>& patches) const
{
  patches = patches_;
}

void MultiScaleDetectExtract::getPatches(cv::Mat&patches_out) const
{
  if (patches_.empty())
    return;
  if (patches_out.empty())
    patches_out = cv::Mat(1, patches_.front().cols * 10, patches_.front().type());
  cv::Mat temp_patch;
  BOOST_FOREACH(cv::Mat patch, patches_)
        {
          if (temp_patch.empty())
            patch.copyTo(temp_patch);
          else
            temp_patch.push_back(patch);
          if (temp_patch.rows == patches_out.cols)
          {
            cv::Mat tt = temp_patch.t();
            if (patches_out.rows == 1)
              tt.copyTo(patches_out);
            else
              patches_out.push_back(tt);
            temp_patch = Mat();
          }
        }
}

void MultiScaleDetectExtract::detectAndExtract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                               cv::Mat& descriptors, const cv::Mat& mask)
{
  watch_ = StopWatch();
  std::vector<float> scales(n_levels_, 1);
  keypoints.clear();
  working_mats_[zoom_level_] = image;
  working_masks_[zoom_level_] = mask;
  //  omp_set_num_threads(1);
  //#pragma omp parallel for
  for (int i = 0; i < n_levels_; i++)
  {
    if (i != zoom_level_)
    {
      watch_.startLap("scaling");
      float scale = 1 / std::pow(scale_factor_, i - zoom_level_);
      scales[i] = scale;
      cv::resize(working_mats_[zoom_level_], working_mats_[i], cv::Size(), scale, scale, cv::INTER_AREA);
      if (!mask.empty())
        cv::resize(working_masks_[zoom_level_], working_masks_[i], cv::Size(), scale, scale, cv::INTER_AREA);
      watch_.stopLap("scaling");
    }

    std::vector<cv::KeyPoint>& kpts = working_kpts_[i];
    cv::Mat& desc = working_descriptors_[i];
    watch_.startLap("detecting");
    feature_detectors_[i]->detect(working_mats_[i], kpts, working_masks_[i]);
    watch_.stopLap("detecting");
    watch_.startLap("compute");
    // +1 as we need to give the step of the integral image
    rbriefs_[i]->setStepSize(working_mats_[i].cols + 1);
    rbriefs_[i]->compute(working_mats_[i], kpts, desc);
    watch_.stopLap("compute");
    if (i != zoom_level_)
    {
      float scale = 1 / scales[i];
      for (size_t j = 0; j < kpts.size(); j++)
      {
        kpts[j].pt *= scale;
        kpts[j].octave = i;
      }

    }
  }
  for (int i = 0; i < n_levels_; i++)
  {
    if (get_patches_)
    {
      std::vector<cv::Mat> patches;
      GetPatches(Size(31, 31), working_mats_[i], working_kpts_[i], patches);
      patches_.insert(patches_.end(), patches.begin(), patches.end());
    }
    keypoints.insert(keypoints.end(), working_kpts_[i].begin(), working_kpts_[i].end());
    if (i == 0)
    {
      working_descriptors_[i].copyTo(descriptors);
    }
    else
      descriptors.push_back(working_descriptors_[i]);
  }

}
void MultiScaleDetectExtract::debugDisplay() const
{
  for (int i = 0; i < n_levels_; i++)
  {
    std::stringstream ss;
    ss << "scale " << 1 / std::pow(scale_factor_, i);
    imshow(ss.str(), working_mats_[i]);
  }
}

}
