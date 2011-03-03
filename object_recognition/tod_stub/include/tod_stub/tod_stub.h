/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * $Id$
 * Author: Ethan Rublee
 */
#ifndef TOD_STUB_INTERFACES_H_
#define TOD_STUB_INTERFACES_H_

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>

#include <pcl_visualization/pcl_visualizer.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/program_options.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <string>

#include <stdexcept>

namespace tod_stub
{
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud_t;

struct FrameData
{
  cv::Mat image; //!<rgb data
  Cloud_t cloud; //!<dense 3d data
  cv::Mat K, D; //!<camera info
};
struct TrainData
{
  FrameData frame; //!< Camera data
  cv::Mat R, T;//!<transform from camera to the object coordinate system
  std::string object_id; //!<unique name for the object
};

struct Result
{
  cv::Mat R, T; //pose of object
  std::string object_id; //unique name for the object
  double confidence; //!< confidence in this result, [0,1] 1 being 100% confidence
};

enum tod_exception
{
  QUIT, ERROR, BAD_POSE
};
/**
 *
 */
class Common
{
public:
  virtual ~Common()
  {
  }
  virtual void onInit() = 0;
  virtual void onFinish() = 0;

};
class Trainer : public Common
{
public:
  virtual ~Trainer()
  {
  }

  virtual void process(const TrainData& data) throw (tod_exception) = 0;

};

class Detector : public Common
{
public:
  virtual ~Detector()
  {
  }
  virtual void detect(const FrameData& data, std::vector<Result>& results) throw (tod_exception) = 0;
};

class ImagePointsCamera
{
public:
  sensor_msgs::ImageConstPtr img;
  sensor_msgs::CameraInfoConstPtr camera_info;
  Cloud_t::ConstPtr points2;
  bool full() const;
  void clear();
  void convert(FrameData& frame) const;
  struct Processor
  {
    ~Processor()
    {
    }
    virtual void operator()(const ImagePointsCamera& ipc) = 0;
  };

  typedef boost::function1<void, const ImagePointsCamera&> ProcessorCallable;

  static void ProcessBag(const rosbag::Bag& bag, ProcessorCallable c, const std::vector<std::string>& topics =
      std::vector<std::string>());

protected:

};

class PoseDrawer3d
{
public:
  PoseDrawer3d();
  
  /** \brief Set the pose to be drawn
   * @param r 3x3 rotation matrix
   * @param t 3x1 translation
   * @param prefix an option prefix for the pose, if you might want to draw more than one pose
   */
  void setRT(cv::Mat r, cv::Mat t, const std::string& prefix);

  void drawResults(const std::vector<Result>& results);
  
  /** This should be called by the pcl_visualization::CloudViewer and will
   * draw the given object pose.
   * @param viewer a viewer that this will draw to.
   */
  void operator()(pcl_visualization::PCLVisualizer& viewer);
private:
  cv::Mat t_;
  cv::Mat r_;
  std::string prefix_;
  bool update_;
  boost::mutex set_mtx_;
  std::vector<Result> results_, previous_results_;
};


struct Options
{
  std::string bag_file;
  std::string config_file;
  //std::string tod_config_file;
  boost::program_options::options_description desc;
};

int options(int ac, char ** av, Options& opts);

}

#endif /* TOD_STUB_INTERFACES_H_ */
