/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 Author: Ethan Rublee, angle functions Gary Bradski, optimizations by
 Vincent Rabaud
 *********************************************************************/

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "rbrief/StopWatch.h"
#include "rbrief/RBrief.h"

using namespace cv;
namespace rbrief
{
namespace
{
static const int HALF_KERNEL = RBriefDescriptorExtractor::KERNEL_SIZE / 2;
static const int RADIUS = BriefDescriptorExtractor::PATCH_SIZE / 2 - 4;
static const int R2 = RADIUS * RADIUS;
static const int C_X = 0;
static const int C_Y = 1;
static const int C_TOP = 0;
static const int C_BOTTOM = 1;
static const int CIRCLE_SLICES_SIZE = RADIUS * 2;

inline int smoothedSum(const int *center, const int* int_diff)
{
  // Points in order 01
  //                 32
  return *(center + int_diff[2]) - *(center + int_diff[3]) - *(center + int_diff[1]) + *(center + int_diff[0]);
}

inline char smoothed_comparison(const int * center, const int* diff, int l, int m)
{
  return (smoothedSum(center, diff + l) < smoothedSum(center, diff + l + 4)) ? (1 << m) : 0;
}
}
class RBriefDescriptorExtractor::RBriefPatterns
{
public:
  static const int N_ANGLES = 30;

  /** Constructor
   * Add +1 to the step as this is the step of the integral image, not image
   * @param sz
   * @param normalized_step
   * @return
   */
  RBriefPatterns(int sz, unsigned int normalized_step_size) :
    normalized_step_(normalized_step_size)
  {
    relative_patterns_.resize(N_ANGLES);
    for (int i = 0; i < N_ANGLES; i++)
      generate_relative_pattern(i, sz, relative_patterns_[i]);
  }

  /** Generate the patterns and relative patterns
   * @param sz
   * @param normalized_step
   * @return
   */
  static std::vector<cv::Mat> generate_rotated_patterns(int sz)
  {
    std::vector<cv::Mat> rotated_patterns(N_ANGLES);
    cv::Mat_<cv::Vec2i> pattern = cv::Mat(
                                          512,
                                          1,
                                          CV_32SC2,
                                          sz == 48 ? BIT_PATTERN_48 : sz == 32 ? BIT_PATTERN_32 : sz == 16
                                              ? BIT_PATTERN_16 : BIT_PATTERN_LEARNED);
    for (int i = 0; i < N_ANGLES; i++)
    {
      const cv::Mat rotation_matrix = GetRotationMat(i);
      transform(pattern, rotated_patterns[i], rotation_matrix);
      // Make sure the pattern is now one channel, and 512*2
      rotated_patterns[i] = rotated_patterns[i].reshape(1, 512);
    }
    return rotated_patterns;
  }

  /** Compute the brief pattern for a given keypoint
   * @param angle the orientation of the keypoint
   * @param sum the integral image
   * @param pt the keypoint
   * @param descriptor the descriptor
   */
  void compute(float angle, const Mat& sum, const KeyPoint& pt, unsigned char * desc) const
  {
    // Compute the pointer to the center of the feature
    int img_y = (int)(pt.pt.y + 0.5);
    int img_x = (int)(pt.pt.x + 0.5);
    const int * center = reinterpret_cast<const int *> (sum.ptr(img_y)) + img_x;
    // Compute the pointer to the absolute pattern row
    const int * diff = relative_patterns_[angle_2_index(angle)].ptr<int> (0);
    for (int i = 0, j = 0; i < 32; ++i, j += 64)
    {
      desc[i] = smoothed_comparison(center, diff, j, 7) | smoothed_comparison(center, diff, j + 8, 6)
          | smoothed_comparison(center, diff, j + 16, 5) | smoothed_comparison(center, diff, j + 24, 4)
          | smoothed_comparison(center, diff, j + 32, 3) | smoothed_comparison(center, diff, j + 40, 2)
          | smoothed_comparison(center, diff, j + 48, 1) | smoothed_comparison(center, diff, j + 56, 0);
    }
  }

  /** Compare the currently used normalized step of the integral image to a new one
   * @param integral_image the integral we want to use the pattern on
   * @return true if the two steps are equal
   */
  bool compareNormalizedStep(const cv::Mat & integral_image) const
  {
    return (normalized_step_ == integral_image.step1());
  }

  /** Compare the currently used normalized step of the integral image to a new one
   * @param step_size the normalized step size to compare to
   * @return true if the two steps are equal
   */
  bool compareNormalizedStep(unsigned int normalized_step_size) const
  {
    return (normalized_step_ == normalized_step_size);
  }

private:
  static inline int angle_2_index(float angle)
  {
    return (angle / 360) * N_ANGLES;
  }

  void generate_relative_pattern(int angle_idx, int sz, cv::Mat & relative_pattern)
  {
    // Create the relative pattern
    relative_pattern.create(512, 4, CV_32SC1);
    int * relative_pattern_data = reinterpret_cast<int*> (relative_pattern.data);
    // Get the original rotated pattern
    const int * pattern_data;
    switch (sz)
    {
      case 16:
        pattern_data = reinterpret_cast<int*> (RotatedPatterns16[angle_idx].data);
        break;
      case 32:
        pattern_data = reinterpret_cast<int*> (RotatedPatterns32[angle_idx].data);
        break;
      case 48:
        pattern_data = reinterpret_cast<int*> (RotatedPatterns48[angle_idx].data);
        break;
      default:
        pattern_data = reinterpret_cast<int*> (RotatedPatternsLearned[angle_idx].data);
        break;
    }

    int half_kernel = RBriefDescriptorExtractor::KERNEL_SIZE / 2;
    for (unsigned int i = 0; i < 512; ++i)
    {
      int center = *(pattern_data + 2*i) + normalized_step_ * (*(pattern_data + 2*i+1));
      // Points in order 01
      //                 32
      // +1 is added for certain coordinates for the integral image
      *(relative_pattern_data++) = center - half_kernel - half_kernel * normalized_step_;
      *(relative_pattern_data++) = center + (half_kernel + 1) - half_kernel * normalized_step_;
      *(relative_pattern_data++) = center + (half_kernel + 1) + (half_kernel + 1) * normalized_step_;
      *(relative_pattern_data++) = center - half_kernel + (half_kernel + 1) * normalized_step_;
    }
  }

  static Mat GetRotationMat(int angle_idx)
  {
    float a = float(angle_idx) / N_ANGLES * CV_PI * 2;
    Mat ar = (Mat_<float> (2, 2) << cos(a), -sin(a), sin(a), cos(a));
    return ar;
  }

  /** Contains the relative patterns (rotated ones in relative coordinates)
   */
  std::vector<cv::Mat_<int> > relative_patterns_;

  /** The step of the integral image
   */
  size_t normalized_step_;

  /** Pattern loaded from the include files
   */
  static std::vector<cv::Mat> RotatedPatterns16;
  static std::vector<cv::Mat> RotatedPatterns32;
  static std::vector<cv::Mat> RotatedPatterns48;
  static std::vector<cv::Mat> RotatedPatternsLearned;
  static int BIT_PATTERN_16[256 * 4]; //number of tests * 4 (x1,y1,x2,y2)
  static int BIT_PATTERN_32[256 * 4]; //number of tests * 4 (x1,y1,x2,y2)
  static int BIT_PATTERN_48[256 * 4]; //number of tests * 4 (x1,y1,x2,y2)
  static int BIT_PATTERN_LEARNED[256 * 4]; //number of tests * 4 (x1,y1,x2,y2)

};

std::vector<cv::Mat> RBriefDescriptorExtractor::RBriefPatterns::RotatedPatterns16 =
    RBriefPatterns::generate_rotated_patterns(16);
std::vector<cv::Mat> RBriefDescriptorExtractor::RBriefPatterns::RotatedPatterns32 =
    RBriefPatterns::generate_rotated_patterns(32);
std::vector<cv::Mat> RBriefDescriptorExtractor::RBriefPatterns::RotatedPatterns48 =
    RBriefPatterns::generate_rotated_patterns(48);
std::vector<cv::Mat> RBriefDescriptorExtractor::RBriefPatterns::RotatedPatternsLearned =
    RBriefPatterns::generate_rotated_patterns(41);

//this is the definition for BIT_PATTERN
#include "pattern16.i"
#include "pattern32.i"
#include "pattern.i"
#include "learned.i"

// bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.
RBriefDescriptorExtractor::RBriefDescriptorExtractor() :
  flags_(USE_DEFAULT), patch_size(PATCH_48), patterns_(NULL)
{

}
RBriefDescriptorExtractor::RBriefDescriptorExtractor(RBriefDescriptorExtractor::PatchSize patch_size) :
  flags_(USE_DEFAULT), patch_size(patch_size), patterns_(NULL)
{

}

RBriefDescriptorExtractor::RBriefDescriptorExtractor(const RBriefDescriptorExtractor& rhs)
{
  //  if(rhs.patterns_)
  //    patterns_ = new RBriefPatterns(rhs.patterns_);
}
RBriefDescriptorExtractor& RBriefDescriptorExtractor::operator=(const RBriefDescriptorExtractor& rhs)
{
  //  if(rhs.patterns_ && (&rhs) != this){
  //    patterns_ = new RBriefPatterns(rhs.patterns_);
  //  }
  return *this;

}
RBriefDescriptorExtractor::~RBriefDescriptorExtractor()
{
  delete patterns_;
}
void RBriefDescriptorExtractor::setPrecomputedIntegralImage(const cv::Mat& sum)
{
  sum_ = sum;
}
void RBriefDescriptorExtractor::setSteerer(cv::Ptr<KeypointSteerer> steerer)
{
  steerer_ = steerer;
}
void RBriefDescriptorExtractor::setFlags(int flags)
{
  flags_ = flags;
}

int RBriefDescriptorExtractor::descriptorSize() const
{
  return BYTES;
}
int RBriefDescriptorExtractor::descriptorType() const
{
  return CV_8UC1;
}

void RBriefDescriptorExtractor::read(const FileNode&)
{
  //TODO: rbrief read
}
void RBriefDescriptorExtractor::write(FileStorage&) const
{
  //todo: rbrief write
}

void RBriefDescriptorExtractor::removeBorderKeyPoints(std::vector<cv::KeyPoint>& keypoints, cv::Size image_size) const
{
  //Remove keypoints very close to the border
  // removeBorderKeypoints(keypoints, image_size, patch_size + KERNEL_SIZE / 2 + 2);
}

void RBriefDescriptorExtractor::setPatchSize(PatchSize sz)
{
  patch_size = sz;
}
#define TIMING_INFO 1

#if TIMING_INFO
#define TIMER_START(x) (timer.start(x))
#define TIMER_STOP(x) (timer.stop(x))
#else
#define TIMER_START(x) ()
#define TIMER_STOP(x) ()
#endif

StopWatch RBriefDescriptorExtractor::timer;
void RBriefDescriptorExtractor::computeImpl(const Mat& image, std::vector<KeyPoint>& keypoints, Mat& descriptors) const
{
  TIMER_START("total");
  //convert to grayscale if more than one color
  Mat grayImage = image;
  if (image.type() != CV_8UC1)
    cvtColor(image, grayImage, CV_BGR2GRAY);

  TIMER_START("integral image");
  // Construct integral image for fast smoothing (box filter)
  Mat sum;
  if (sum_.empty())
    integral(grayImage, sum, CV_32S);
  else
    sum = sum_; //a precomputed integral image was set
  TIMER_STOP("integral image");

  //Remove keypoints very close to the border
  if (USE_REMOVE_BORDER_KEYPOINTS & flags_)
  {
    removeBorderKeyPoints(keypoints, image.size());
  }

  if (steerer_)
  {
    TIMER_START("steering");
    steerer_->steer(keypoints, grayImage, sum);
    TIMER_STOP("steering");
  }

  Ptr<RBriefPatterns> patterns;
  if (patterns_ == NULL || !patterns_->compareNormalizedStep(sum))
  {
    TIMER_START("relative_patterns");
    patterns = new RBriefPatterns(patch_size, sum.step1());
    TIMER_STOP("relative_patterns");
  }
  else
  {
    patterns = patterns_;
    patterns.addref();
  }

  TIMER_START("describing");
  Rect img_rect(0, 0, grayImage.cols, grayImage.rows);

  //create the descriptor mat, keypoints.size() rows, BYTES cols
  descriptors = Mat::zeros(keypoints.size(), BYTES, CV_8UC1);

  const int SZ = patch_size + KERNEL_SIZE / 2 + 2;
  const int H_SZ = SZ / 2;
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    const KeyPoint& kpt = keypoints[i];
    float angle = 0;

    Rect roi(int(kpt.pt.x) - H_SZ, int(kpt.pt.y) - H_SZ, SZ, SZ);

    bool not_valid_point = ((USE_REMOVE_BORDER_KEYPOINTS & flags_) == 0) && ((img_rect & roi).area() < (SZ) * (SZ));
    if (!not_valid_point)
    {

      if (steerer_.empty())
      {
        switch (flags_ & (USE_KEYPOINT_ANGLE | USE_ANGLE_ZERO))
        {
          case USE_KEYPOINT_ANGLE | USE_ANGLE_ZERO:
            angle = kpt.angle < 0 ? 0 : kpt.angle;
            break;
          case USE_KEYPOINT_ANGLE:
            angle = kpt.angle;
            break;
          case USE_ANGLE_ZERO:
            angle = 0;
            break;
          default:
            CV_Error(5, "Undefined use of flags for RBrief");
        }
      }
      else
        angle = kpt.angle;

      //look up the test pattern by angle
      patterns->compute(angle, sum, kpt, descriptors.ptr(i));
    }
  }
  TIMER_STOP("describing");
  TIMER_STOP("total");
}

/** For efficiency if you know the size of your images, call this function to set the step size
 * of the integral image
 * @param step_size the step size of the integral image
 */
void RBriefDescriptorExtractor::setStepSize(unsigned int step_size)
{
  if ((patterns_ != NULL) && (!patterns_->compareNormalizedStep(step_size)))
  {
    delete patterns_;
    patterns_ = NULL;
  }
  if (patterns_ == NULL)
  {
    patterns_ = new RBriefPatterns(patch_size, step_size);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//A varient of the above that might be as fast but more accurate
#define MinGradThresh 30.0
#define CourseBinsDegrees 10
#define FineBinsDegrees (CourseBinsDegrees/2)
#define NumBinsFine (int)(1 + 360/FineBinsDegrees)
#define NumBinsCourse (int)(360/CourseBinsDegrees)
float gradientAngleDegrees2(const cv::Mat& I, vector<Mat>& working_mats)
{
  working_mats.resize(4);

  cv::Mat& X = working_mats[0];
  cv::Mat& Y = working_mats[1];
  cv::Mat& mag = working_mats[2];
  cv::Mat& angle = working_mats[3];
  Scharr(I, X, CV_32F, 1, 0);
  Scharr(I, Y, CV_32F, 0, 1);
  cartToPolar(X, Y, mag, angle, true);

  Mat_<float>::iterator M = mag.begin<float> (), MEnd = mag.end<float> ();
  Mat_<float>::iterator A = angle.begin<float> ();
  std::vector<int> ha(NumBinsFine, 0), hb(NumBinsCourse, 0); //Store histogram counts
  for (; M != MEnd; ++M, ++A)
  {
    if (*M > MinGradThresh)
    {
      int indexa = (int)((*A) / FineBinsDegrees);
      ha[indexa] += 2; //Accumulate into histogram every 5 degrees
    }
  }
  //Combine into a histogram every 10 degrees, special case at ends
  int max_index = 0;
  //special case at beginning
  hb[0] = ((ha[NumBinsFine - 1] + ha[NumBinsFine - 2] + ha[1]) >> 1) + ha[0];
  //main case
  int max_amount = hb[max_index];
  int i = 1, ja = 2;
  for (; i < NumBinsCourse - 1; ++i, ja += 2)
  {
    hb[i] = ((ha[ja - 1] + ha[ja + 1]) >> 1) + ha[ja]; //Histogram averaging the side bins
    if (hb[i] > max_amount) //Keep track of the max location index per 10 degrees

    {
      max_amount = hb[i];
      max_index = i;
    }
  }
  //special case at end
  hb[i] = ((ha[ja + 2] + ha[ja + 1] + ha[ja - 1]) >> 1) + ha[ja];
  if (hb[i] > max_amount)
  {
    max_amount = hb[i];
    max_index = i;
  }
  return max_index * CourseBinsDegrees;
}

void steerKeyPoints2(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image)
{
  vector<Mat> working_mats;
  const float k = 48 / 2;
  const float half_k = k / 2.0f;
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    Point pt = keypoints[i].pt;
    Rect roi(pt.x - half_k, pt.y - half_k, k, k);
    //get a patch at the keypoint
    Mat patch = image(roi);
    //find the angle based on gradient
    keypoints[i].angle = gradientAngleDegrees2(patch, working_mats); //found angle here
    //    if(i%20)
    //    	std::cout << keypoints[i].angle << endl;
  }
}
void steerKeyPoints(std::vector<cv::KeyPoint>& keypoints, float angle)
{
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    keypoints[i].angle = angle;
  }
}

/**
 *  create a mask of values that resemble a circle and are blurred
 *  to weight down the edges
 */
Mat circlularWeights(cv::Size sz, int radius)
{
  Mat mask = Mat::zeros(sz, CV_32F);
  Point center = Point(sz.width / 2, sz.height / 2);
  cv::circle(mask, center, radius * 2.0 / 4.0, Scalar(1.0), -1, CV_AA);
  GaussianBlur(mask, mask, Size(radius + 1, radius + 1), 0, 0);
  return mask;
}

struct GAWorkingMats
{
  GAWorkingMats(cv::Size sz, int radius) :
    circle_weight(circlularWeights(sz, radius))
  {
    namedWindow("circle weight", CV_WINDOW_KEEPRATIO);
    imshow("circle weight", circle_weight);
    Mat dw;
    circle_weight.convertTo(dw, CV_8U, 255);
    cv::resize(dw.clone(), dw, Size(300, 300), 0, 0, cv::INTER_AREA);
    imwrite("circle_weight.png", dw);
  }
  cv::Mat I, X, Y, mag, angle, mask, circle_weight;
};

void steerKeyPoints(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image_, const cv::Mat & integral_image)
{

  Size patch_size(48, 48);
  Size blur_sz(5, 5);
  Ptr<KeypointSteerer> steerer(new GradientSteerer(GradientSteerer::BIN, patch_size, blur_sz));
  steerer->steer(keypoints, image_);
}
struct GradientSteerer::GradientSteerer_Impl
{
  Method method;
  Size patch_sz, blur_sz;
  GradientSteerer_Impl(Method method, Size patch_sz, Size blur_sz, bool debug) :
    method(method), patch_sz(patch_sz), blur_sz(blur_sz), debug(debug)
  {
    k = patch_sz.width;
    half_k = k / 2;
    circlular_weights = circlularWeights(Size(k, k), half_k);
    if (debug)
    {
      namedWindow("circle weight", CV_WINDOW_KEEPRATIO);
      imshow("circle weight", circlular_weights);
      Mat dw;
      circlular_weights.convertTo(dw, CV_8U, 255);
      cv::resize(dw.clone(), dw, Size(300, 300), 0, 0, cv::INTER_AREA);
      imwrite("circlular_weights.png", dw);
    }
  }

  void checkVals()
  {
    CV_Assert(patch_sz.width == patch_sz.height)
      ;
    CV_Assert(blur_sz.height == blur_sz.width)
      ;
    CV_Assert(blur_sz.height % 2 != 0)
      ;

  }
  Mat image, X, Y, sX, sY, mag, angle, smag, sangle, circlular_weights;
  int k, half_k;
  bool debug;
  void steer(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image_)
  {
    checkVals();
    //operate on whole image as keypoints and patches may overlap,
    //and these functions may be optimized for doing all at once
    //rather than in inner loop
    GaussianBlur(image_, image, blur_sz, 0, 0); // blur the image
    Scharr(image, X, CV_32F, 1, 0); //find the X derivative
    Scharr(image, Y, CV_32F, 0, 1); //find the Y derivative
    cartToPolar(X, Y, mag, angle, true); //get the magnitude and angle for the derivative vectors.
    Rect img_rect(0, 0, image.cols, image.rows);
    for (size_t i = 0; i < keypoints.size(); i++)
    {
      Point pt = keypoints[i].pt;
      float & ka = keypoints[i].angle; //grab reference to angle for modification later
      Rect roi(pt.x - half_k, pt.y - half_k, k, k);
      if ((img_rect & roi).area() < roi.area())
      {
        ka = 0;
        continue; // test if the roi is completely inside the image, if not continue
      }
      //get a patch at the keypoint
      Mat patch = image(roi);
      sX = X(roi);
      sY = Y(roi);
      //weight down the magnitude in the area around the keypoint, as we want this to be invariant to rotation
      smag = mag(roi).mul(circlular_weights); //weight the magnitudes as these are use for thresholding
      sangle = angle(roi);
      switch (method)
      {
        case GradientSteerer::MAX:
          ka = angleMax(sX, sY, smag, sangle);
          break;
        case GradientSteerer::BIN:
          ka = angleBin(sX, sY, smag, sangle);
          break;
      }
      //   if(i%20)
      //            std::cout << keypoints[i].angle << endl;
    }
  }
  float angleBin(const cv::Mat& X, const cv::Mat& Y, const cv::Mat& mag, const cv::Mat& angle)
  {
    int ha[NumBinsCourse] = {0}; //histogram of angles

    //    //iterators
    //    Mat_<float>::const_iterator M = mag.begin<float> (), MEnd = mag.end<float> ();
    //    Mat_<float>::const_iterator A = angle.begin<float> ();


    for (int i = 0; i < mag.size().area(); i++)
    {
      float A = angle.at<float> (i / angle.rows, i % angle.cols);
      float M = mag.at<float> (i / angle.rows, i % angle.cols);
      // std::cout << A << " " << i << std::endl;
      //TODO come up with more intelligent threshholding
      if (M > MinGradThresh)
      {

        int indexa = ((int)(A) / CourseBinsDegrees);
        ha[indexa] += 3; //Accumulate into histogram every 10 degrees
        ha[indexa + 1 < NumBinsCourse ? indexa + 1 : 0] += 1; //add 1 to adjacent bins
        ha[indexa - 1 > 0 ? indexa - 1 : NumBinsCourse] += 1; //for spillage reasons.
      }
    }
    return CourseBinsDegrees * (std::max_element(ha, ha + NumBinsCourse) - ha);
  }

  float angleMax(const cv::Mat& X, const cv::Mat& Y, const cv::Mat& mag, const cv::Mat& angle)
  {
    double maxMag;
    Point loc;
    minMaxLoc(mag, NULL, &maxMag, NULL, &loc); //find the max magnitued and get its location
    return angle.at<float> (loc); //return the max magnitude angle
  }

};
GradientSteerer::GradientSteerer(Method method, Size patch_sz, Size blur_sz, bool debug) :
  impl_(new GradientSteerer_Impl(method, patch_sz, blur_sz, debug))
{

}
GradientSteerer::~GradientSteerer()
{
  delete impl_;
}
void GradientSteerer::steer(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image, const cv::Mat& integral_image) const
{
  impl_->steer(keypoints, image);
}

GradientSteerer::GradientSteerer(const GradientSteerer& rhs) :
  impl_(new GradientSteerer_Impl(rhs.impl_->method, rhs.impl_->patch_sz, rhs.impl_->blur_sz, rhs.impl_->debug))
{

}
GradientSteerer&
GradientSteerer::operator=(const GradientSteerer& rhs)
{
  if (this == &rhs) //self assignment
    return *this;
  *impl_ = GradientSteerer_Impl(rhs.impl_->method, rhs.impl_->patch_sz, rhs.impl_->blur_sz, rhs.impl_->debug);
  return *this;
}

void GradientSteerer::setMethod(Method method)
{
  impl_->method = method;
}
GradientSteerer::Method GradientSteerer::getMethod() const
{
  return impl_->method;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

UniformSteerer::UniformSteerer(float angle) :
  angle_(angle)
{

}
void UniformSteerer::steer(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image, const cv::Mat &integral_image) const
{
  steerKeyPoints(keypoints, angle_);
}
void UniformSteerer::setAngle(float angle)
{
  angle_ = angle;
}
float UniformSteerer::getAngle() const
{
  return angle_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{

/** Simple function that returns the area in the rectangle x1<=x<=x2, y1<=y<=y2 given an integral image
 * @param integral_image
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
template<typename SumType>
  inline SumType integral_rectangle(const SumType * val_ptr, std::vector<int>::const_iterator offset)
  {
    return *(val_ptr + *offset) - *(val_ptr + *(offset + 1)) - *(val_ptr + *(offset + 2)) + *(val_ptr + *(offset + 3));
  }

template<typename SumType>
  void IC_Angle_Integral(const cv::Mat& integral_image, const int half_k, KeyPoint& kpt,
                         const std::vector<int> &horizontal_offsets, const std::vector<int> &vertical_offsets)
  {
    SumType m_01 = 0, m_10 = 0;

    // Go line by line in the circular patch
    std::vector<int>::const_iterator horizontal_iterator = horizontal_offsets.begin(), vertical_iterator =
        vertical_offsets.begin();
    const SumType* val_ptr = &(integral_image.at<SumType> (kpt.pt.y, kpt.pt.x));
    for (int uv = 1; uv <= half_k; ++uv)
    {
      // Do the horizontal lines
      m_01 += uv * (-integral_rectangle(val_ptr, horizontal_iterator) + integral_rectangle(val_ptr,
                                                                                           horizontal_iterator + 4));
      horizontal_iterator += 8;

      // Do the vertical lines
      m_10 += uv * (-integral_rectangle(val_ptr, vertical_iterator)
          + integral_rectangle(val_ptr, vertical_iterator + 4));
      vertical_iterator += 8;
    }

    float x = m_10;
    float y = m_01;
    kpt.angle = fastAtan2(y, x);
  }

template<typename PatchType, typename SumType>
  void IC_Angle(const cv::Mat& image, const int half_k, KeyPoint& kpt, const std::vector<int> & u_max)
  {
    SumType m_01 = 0, m_10 = 0/*, m_00 = 0*/;

    const PatchType* val_center_ptr_plus = &(image.at<PatchType> (kpt.pt.y, kpt.pt.x)), *val_center_ptr_minus;
    // Treat the center line differently, v=0
    {
      const PatchType* val = val_center_ptr_plus - half_k;
      for (int u = -half_k; u <= half_k; ++u, ++val)
        m_10 += u * (SumType)(*val);
    }
    // Go line by line in the circular patch
    val_center_ptr_minus = val_center_ptr_plus - image.step1();
    val_center_ptr_plus += image.step1();
    for (int v = 1; v <= half_k; ++v, val_center_ptr_plus += image.step1(), val_center_ptr_minus -= image.step1())
    {
      // The beginning of the two lines
      const PatchType* val_ptr_plus = val_center_ptr_plus - u_max[v];
      const PatchType* val_ptr_minus = val_center_ptr_minus - u_max[v];

      // Proceed over the two lines
      SumType v_sum = 0;
      for (int u = -u_max[v]; u <= u_max[v]; ++u, ++val_ptr_plus, ++val_ptr_minus)
      {
        SumType val_plus = *val_ptr_plus, val_minus = *val_ptr_minus;
        v_sum += (val_plus - val_minus);
        m_10 += u * (val_plus + val_minus);
      }
      m_01 += v * v_sum;
    }

    float x = m_10;// / float(m_00);// / m_00;
    float y = m_01;// / float(m_00);// / m_00;
    kpt.angle = fastAtan2(y, x);
  }
}

IntensityCentroid::IntensityCentroid(int half_k) :
  half_patch_size_(half_k)
{
}

void IntensityCentroid::steer(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image, const cv::Mat& integral_image) const
{
  // pre-compute the end of a row in a circular patch
  std::vector<int> u_max(half_patch_size_ + 1);
  for (int v = 0; v <= half_patch_size_ * sqrt(2) / 2 + 1; ++v)
    u_max[v] = std::floor(sqrt(half_patch_size_ * half_patch_size_ - v * v) + 0.5);

  // Make sure we are symmetric
  for (int v = half_patch_size_, v_0 = 0; v >= half_patch_size_ * sqrt(2) / 2; --v)
  {
    while (u_max[v_0] == u_max[v_0 + 1])
      ++v_0;
    u_max[v] = v_0;
    ++v_0;
  }

  // If using the integral image, some offsets will be computed for speed
  std::vector<int> horizontal_offsets(8 * half_patch_size_), vertical_offsets(8 * half_patch_size_);
  if (!integral_image.empty())
  {
    int integral_image_step = integral_image.step1();
    for (int v = 1, offset_index = 0; v <= half_patch_size_; ++v)
    {
      // Compute the offsets to use if using the integral image
      for (int signed_v = -v; signed_v <= v; signed_v += 2 * v)
      {
        // the offsets are computed so that we can compute the integral image
        // elem at 0 - eleme at 1 - elem at 2 + elem at 3
        horizontal_offsets[offset_index] = (signed_v + 1) * integral_image_step + u_max[v] + 1;
        vertical_offsets[offset_index] = (u_max[v] + 1) * integral_image_step + signed_v + 1;
        ++offset_index;
        horizontal_offsets[offset_index] = signed_v * integral_image_step + u_max[v] + 1;
        vertical_offsets[offset_index] = -u_max[v] * integral_image_step + signed_v + 1;
        ++offset_index;
        horizontal_offsets[offset_index] = (signed_v + 1) * integral_image_step - u_max[v];
        vertical_offsets[offset_index] = (u_max[v] + 1) * integral_image_step + signed_v;
        ++offset_index;
        horizontal_offsets[offset_index] = signed_v * integral_image_step - u_max[v];
        vertical_offsets[offset_index] = -u_max[v] * integral_image_step + signed_v;
        ++offset_index;
      }
    }
  }

  // Process each keypoint
  BOOST_FOREACH(cv::KeyPoint &keypoint, keypoints)
        {
          const Point & pt = keypoint.pt;
          if ((pt.x - half_patch_size_ < 0) || (pt.x + half_patch_size_ >= image.cols) || (pt.y - half_patch_size_ < 0)
              || (pt.y + half_patch_size_ >= image.rows))
          {
            keypoint.angle = 0;
            continue; // test if the roi is completely inside the image, if not continue
          }
          //get a patch at the keypoint
          if (integral_image.empty())
          {
            switch (image.depth())
            {
              case CV_8U:
                IC_Angle<uchar, int> (image, half_patch_size_, keypoint, u_max);
                break;
              case CV_32S:
                IC_Angle<int, int> (image, half_patch_size_, keypoint, u_max);
                break;
              case CV_32F:
                IC_Angle<float, float> (image, half_patch_size_, keypoint, u_max);
                break;
              case CV_64F:
                IC_Angle<double, double> (image, half_patch_size_, keypoint, u_max);
                break;
            }
          }
          else
          {
            // use the integral image if you can
            switch (integral_image.depth())
            {
              case CV_32S:
              {
                IC_Angle_Integral<int> (integral_image, half_patch_size_, keypoint, horizontal_offsets,
                                        vertical_offsets);
#if 0
                float angle22 = keypoint.angle;
                IC_Angle<uchar, int> (image, half_patch_size_, keypoint, u_max);
                std::cout << angle22 - keypoint.angle << " " << angle22 << " " << keypoint.angle << std::endl;
#endif
                break;
              }
              case CV_32F:
                IC_Angle_Integral<float> (integral_image, half_patch_size_, keypoint, horizontal_offsets,
                                          vertical_offsets);
                break;
              case CV_64F:
                IC_Angle_Integral<double> (integral_image, half_patch_size_, keypoint, horizontal_offsets,
                                           vertical_offsets);
                break;
            }
          }
        }
}

}
