/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 * run with
 * rosparam set base `pwd`
 * rosparam set interval 0.2
 * rosrun tod_detecting recognition_node camera:=/camera/rgb points2:=/camera/depth/points2
 */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/CvBridge.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <pcl_visualization/cloud_viewer.h>
#include <pcl_visualization/pcl_visualizer.h>

#include <rosbag/bag.h>
#include <rbrief/lsh.hpp>
//todo remove dependency on this
#include <posest/pnp_ransac.h>

#include "tod/detecting/Loader.h"
#include "tod/detecting/Cluster.h"
#include "tod/detecting/Matcher.h"
#include "tod/detecting/GuessGenerator.h"
#include "tod/detecting/Parameters.h"
#include "tod/detecting/clustering.h"
#include "tod/training/feature_extraction.h"
#include "tod/core/TexturedObject.h"
#include "tod/training/ros/msgs.h"

using std::string;
using namespace sensor_msgs;

#define foreach BOOST_FOREACH

using std::list;
using std::string;
using std::istream;
using namespace cv;
using namespace tod;

namespace fs = boost::filesystem;

namespace
{
struct Options
{
  std::string image_file;
  std::string base_directory;
  std::string config;
  TODParameters params;
  int verbose;
};
}
namespace tod
{
typedef std::pair<int, int > idx_pair_t;
bool compareGreaterDMatch(const DMatch & lhs, const DMatch & rhs)
{
  return lhs.imgIdx > rhs.imgIdx; //  && lhs.imgIdx == rhs.imgIdx ? lhs.distance < rhs.distance : true;
}
void sortObjectMatchesByView(vector<DMatch> &objectMatches, std::map<int, std::vector<DMatch> > & viewMatches,
                             std::vector<std::pair<int, int > > & viewSizes)
{
  viewMatches.clear();
  viewSizes.clear();
  if (objectMatches.empty())
    return;

  std::sort(objectMatches.begin(), objectMatches.end(), compareGreaterDMatch);
  std::map<int, int > viewsizes_;
  for (size_t i = 0; i < objectMatches.size(); i++)
  {
    viewMatches[objectMatches[i].imgIdx].push_back(objectMatches[i]);
    viewsizes_[objectMatches[i].imgIdx]++;
  }
  foreach (const idx_pair_t & x, viewsizes_)
  {
    viewSizes.push_back(x);
  }
  std::sort(viewSizes.begin(), viewSizes.end(), Matcher::pair_second_greater<idx_pair_t>);
}

void remove(pcl_visualization::PCLVisualizer& viewer, const std::string& prefix)
{
  //remove old coordinate frame
  viewer.removeShape(std::string(prefix + "co_x"));
  viewer.removeShape(std::string(prefix + "co_y"));
  viewer.removeShape(std::string(prefix + "co_z"));
  //viewer.removeShape(prefix + "text");
}
void draw3DPose(pcl_visualization::PCLVisualizer& viewer, const std::string& prefix, const PoseRT& pose)
{
  std::vector<pcl::PointXYZ> coordinate_axis(4);
  remove(viewer, prefix);
  //setup a canonical coordinate system
  cv::Point3d z(0, 0, 0.25);
  cv::Point3d x(0.25, 0, 0);
  cv::Point3d y(0, 0.25, 0);
  cv::Point3d o(0, 0, 0);
  std::vector<cv::Point3d> op(4);
  op[1] = x, op[2] = y, op[3] = z, op[0] = o;
  cv::Mat mop(op);
  //std::cout << r << std::endl;
  cv::Mat R;
  cv::Rodrigues(pose.rvec,R);
  //rotate the points in place
  cv::transform(mop, mop, R);

  //translate the points and add to pcl style point vector
  int idx = 0;
  BOOST_FOREACH(cv::Point3d& p, op)
        {
          p += pose.tvec.at<cv::Point3d> ();
          coordinate_axis[idx++] = pcl::PointXYZ(p.x, p.y, p.z);
        }

  viewer.addLine(coordinate_axis[0], coordinate_axis[1], 1, 0, 0, std::string(prefix + "co_x"));
  viewer.addLine(coordinate_axis[0], coordinate_axis[2], 0, 1, 0, std::string(prefix + "co_y"));
  viewer.addLine(coordinate_axis[0], coordinate_axis[3], 0, 0, 1, std::string(prefix + "co_z"));


 // viewer.addText(prefix, , prefix + "text");
}

class RecognitionNode
{
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo, PointCloud2> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> SynchronizerCameraInfoPointCloud2;

public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_t;
  RecognitionNode(Options opts) :
    sync_sub_(1), time_interval_(0.5), //default to half a second
    cloud_viewer_("train 3d data")
  {
    onInit();
  }

  ~RecognitionNode()
  {
  }
  void onInit()
  {
    ros::NodeHandle & nh = nh_;

    nh.param("interval", time_interval_, 0.5);
    if (time_interval_ < 0.001)
    {
      time_interval_ = 0.001;
    }

    if (!nh.getParam("base", base_dir_))
    {
      ROS_ERROR_STREAM ("Must supply a base directory");
      nh.shutdown();
      return;
    }

    ROS_INFO_STREAM ("Base directory is " << base_dir_);
    nh.param("config", config_file_, (fs::path(base_dir_) / "config.yaml").string());
    {
      FileStorage fs;
      if (!(fs = FileStorage(config_file_, FileStorage::READ)).isOpened())
      {
        ROS_ERROR_STREAM ("Must supply configuration. see newly generated sample.config.yaml");
        FileStorage fs("./sample.config.yaml", FileStorage::WRITE);
        fs << TODParameters::YAML_NODE_NAME;
        TODParameters::CreateSampleParams().write(fs);
        nh.shutdown();
        return;
      }
      else
        params_.read(fs[TODParameters::YAML_NODE_NAME]);
    }

    // Load the training features we will use to match images to training data
    loader_ = boost::shared_ptr<Loader>(new tod::Loader(base_dir_));
    vector<cv::Ptr<TexturedObject> > objects;
    loader_->readTexturedObjects(objects);

    if (objects.empty())
    {
      ROS_ERROR_STREAM ("Empty base");
      nh.shutdown();
      return;
    }

    base_ = boost::shared_ptr<TrainingBase>(new TrainingBase(objects));

    extractor_ = FeatureExtractor::create(params_.feParams);
    rtMatcher_ = Matcher::create(params_.matcherParams);
    rtMatcher_->add(*base_);
/*
    // Play with LSH, all that could be removed
    LshMatcher* lsh_matcher = dynamic_cast<LshMatcher*>(&(*rtMatcher_->matcher));
    std::vector<LshStats> lsh_stats;
    lsh_matcher->getStats(lsh_stats);
    for (int i = 0; i < lsh_stats.size(); ++i)
    {
      std::cout << "size " << lsh_stats[i].bucket_size_min_ << " " << lsh_stats[i].bucket_size_mean_
          << " " << lsh_stats[i].bucket_size_max_ << " " << lsh_stats[i].n_buckets_ << std::endl;
    }

    namedWindow("all_patches", CV_WINDOW_KEEPRATIO);


    BOOST_FOREACH(const LshTable&table, lsh_matcher->tables_)
          {
            BOOST_FOREACH(const LshTable::Buckets::value_type& x, table.buckets_)
                  {
                    if (x.second.size() >= 1000)
                    {
                      unsigned int global_x = 0, global_y=0, patch_edge = 33, half_patch_edge = patch_edge/2,
                          square_size = 10;
                      cv::Mat all_patches(square_size*patch_edge, square_size*patch_edge, CV_8UC1);
                      BOOST_FOREACH(unsigned int ind, x.second)
                            {
                              if (global_y>=square_size)
                                break;
                              int image_index, feature_index;
                              lsh_matcher->mergedDescriptors.getLocalIdx(ind, image_index, feature_index);
                              // Find the obejct in which that image is
                              unsigned int obj_ind = 0;
                              waitKey(30);

                              BOOST_FOREACH(const cv::Ptr<TexturedObject> &object, objects)
                                    {
                                      if (image_index > object->getDescriptors().size())
                                      {
                                        image_index -= object->getDescriptors().size();
                                        ++obj_ind;
                                        continue;
                                      }
                                      if (image_index >= object->observations.size())
                                        continue;
                                      // Get the 3d feature of that descriptor
                                      Features2d feature2d = object->observations[image_index].features();
                                      if (feature_index >= feature2d.keypoints.size())
                                        continue;
                                      cv::KeyPoint feature = feature2d.keypoints[feature_index];
                                      if (feature2d.image.empty())
                                      {
                                        string filename = "/home/vrabaud/tod_data/base/" + base_->getObject(obj_ind)->name + "/"
                                            + feature2d.image_name;
                                        feature2d.image = imread(filename, 0);
                                      }
                                      if (feature2d.image.empty())
                                        continue;

                                      int pt_x = feature.pt.x, pt_y = feature.pt.y;
                                      if ((pt_y - (int)half_patch_edge < 0) || (pt_y + (int) half_patch_edge + 1
                                          > feature2d.image.cols) || (pt_x - (int) half_patch_edge < 0) || (pt_x
                                          + (int) half_patch_edge + 1) > feature2d.image.rows)
                                        continue;
                                      cv::Mat patch = feature2d.image(cv::Range(pt_y - half_patch_edge, pt_y
                                          + half_patch_edge+1), cv::Range(pt_x - half_patch_edge, pt_x + half_patch_edge+1));
                                      cv::Mat final_patch = all_patches(cv::Range(patch_edge * global_y, patch_edge
                                          * (global_y + 1)), cv::Range(patch_edge * global_x, patch_edge
                                          * (global_x + 1)));
                                      patch.copyTo(final_patch);
                                      ++global_x;
                                      if (global_x>=square_size) {
                                        global_x = 0;
                                        ++global_y;
                                      }
                                      imshow("all_patches", all_patches);

                                      break;
                                    }
                            }
                    }
                  }
          }
*/

    prev_ = ros::Time::now();

    //may be remapped to : /camera/rgb
    camera_topic_ = nh.resolveName("camera", true);

    //should be remapped to something like: /camera/depth/points2
    points2_topic_ = nh.resolveName("points2", true);

    //subscribe to topics
    point_cloud_sub_.subscribe(nh, points2_topic_, 1);
    image_sub_.subscribe(nh, camera_topic_ + "/image_color", 1);
    gray_sub_.subscribe(nh, camera_topic_ + "/image_mono", 1);
    camera_info_sub_.subscribe(nh, camera_topic_ + "/camera_info", 30);

    sync_sub_.connectInput(image_sub_, gray_sub_, camera_info_sub_, point_cloud_sub_);
    sync_sub_.registerCallback(&RecognitionNode::dataCallback, this);

    ROS_INFO_STREAM ("camera topic is set to " << camera_topic_);
    ROS_INFO_STREAM ("time interval set to attempt " << 1 / time_interval_ << " captures per second." << std::endl);
    ROS_INFO ("init done");

    //add a reference to the cloud_viewer, so that it doesn't copy our pose_drawer_
    cloud_viewer_.runOnVisualizationThread(boost::ref(*this));
  }
  void operator()(pcl_visualization::PCLVisualizer& viewer)
  {

    boost::mutex::scoped_lock(viewer_3d_lock_);
    foreach(const Guess& g, current_guesses_)
          {
            PoseRT frame_pose;
            if(g.getObject().empty())
              continue;
            PoseRT camera_pose = g.getObject()->observations[g.imageIndex].camera().pose;

            cv::composeRT(camera_pose.rvec, camera_pose.tvec, g.pose().rvec, g.pose().tvec, frame_pose.rvec,
                          frame_pose.tvec);
            draw3DPose(viewer, g.getObject()->name, frame_pose);
          }

  }
  void dataCallback(const ImageConstPtr & image, const ImageConstPtr & gray, const CameraInfoConstPtr & camera_info,
                    const PointCloud2ConstPtr & point_cloud)
  {
    boost::mutex::scoped_lock sl(matching_lock_); //don't want any other threads calling this. TODO add threading
    ros::Time n = ros::Time::now();
    if ((n - prev_).toSec() > time_interval_)
    {
      ROS_INFO_STREAM ("got a package, dt=" << (n - prev_).toSec ());
      prev_ = n;
      const cv::Mat cv_image = bridge_.imgMsgToCv(gray, "bgr8");
      tod::Camera camera = tod::fromRosMsg(camera_info);
      pcl::fromROSMsg(*point_cloud, cloud_);
      ROS_INFO_STREAM ("converting messages :" << (ros::Time::now () - prev_).toSec ());
      match(cv_image, cloud_, camera);

    }
    waitKey(5);
  }
  void match(const cv::Mat & image, const cloud_t & cloud, const Camera & camera)
  {

    cloud_viewer_.showCloud(cloud);
    Features3d test;
    ros::Time n = ros::Time::now();
    test.features() = Features2d(camera, image);
    extractor_->detectAndExtract(test.features());
    ROS_INFO_STREAM ("features and descriptors :" << (ros::Time::now () - n).toSec ());
    {
      cv::Mat draw_image;
      test.features().draw(draw_image);
      imshow("features", draw_image);
      waitKey(100);
    }

    ROS_INFO_STREAM ("found " << test.features ().keypoints.size () << " features");
    n = ros::Time::now();
    rtMatcher_->match(test.features().descriptors);
    ROS_INFO_STREAM ("matching :" << (ros::Time::now () - n).toSec ());
#if DISPLAY
    Matcher::drawMatches(*base_, rtMatcher_, test.features().image, test.features().keypoints, base_dir_);
#endif
    n = ros::Time::now();

    std::vector<std::pair<int, int > > labels_sizes;

    rtMatcher_->getLabelSizes(labels_sizes);
    vector<int > objectIds;
    base_->getObjectIds(objectIds);
    vector<DMatch> objectMatches;

    cv::Mat projection;

    // Deal with the pose estimation
    vector<Guess> guesses_all;
    foreach (const idx_pair_t & x, labels_sizes)
    {
      if (x.second < params_.guessParams.minInliersCount)
        break;
      rtMatcher_->getObjectMatches(x.first, objectMatches);
      std::map<int, std::vector<DMatch> > viewMatches;
      std::vector<std::pair<int, int > > viewSizes;
      sortObjectMatchesByView(objectMatches, viewMatches, viewSizes);
      GuessGenerator generator(params_.guessParams);
      vector<Guess> guesses;

      for (size_t i = 0; i < viewSizes.size() /* && i < 3 TODO parameterize this*/; i++)
      {
        generator.calculateGuesses(base_->getObject(x.first), viewMatches[viewSizes[i].first],
                                   test.features().keypoints, test.features().image, guesses);
        guesses_all.insert(guesses_all.end(), guesses.begin(), guesses.end());
      }

    }

    ROS_INFO_STREAM ("pose fitting :" << (ros::Time::now () - n).toSec ());
    n = ros::Time::now();
    if (guesses_all.size())
    {
      foreach (const Guess & guess, guesses_all)
      {
        guess.draw(projection, 0);
      }
      {
        //post the guesses to the drawing loop.
        boost::mutex::scoped_lock(viewer_3d_lock_);
        current_guesses_ = guesses_all;
      }
    }
    if (!projection.empty())
    {
      namedWindow("projection", CV_WINDOW_KEEPRATIO);
      imshow("projection", projection);

    }
    waitKey(100);
  }
private:

  ImageSubscriber image_sub_, gray_sub_;
  PointCloud2Subscriber point_cloud_sub_;
  CameraInfoSubscriber camera_info_sub_;
  SynchronizerCameraInfoPointCloud2 sync_sub_;
  ros::NodeHandle nh_;

  string bag_filename_;
  string camera_topic_;
  string points2_topic_;

  ros::Time prev_;
  double time_interval_;
  rosbag::Bag bag_;

  sensor_msgs::CvBridge bridge_;

  boost::shared_ptr<Loader> loader_;
  boost::shared_ptr<TrainingBase> base_;

  std::string base_dir_;
  std::string config_file_;

  TODParameters params_;

  Ptr<FeatureExtractor> extractor_;
  cv::Ptr<Matcher> rtMatcher_;

  cloud_t cloud_;
  boost::mutex matching_lock_;

  pcl_visualization::CloudViewer cloud_viewer_;
  boost::mutex viewer_3d_lock_;
  std::vector<Guess> current_guesses_;

  //create a thread for each sublist
  // boost::shared_ptr<boost::thread> thread;
};

}

int main(int argc, char **argv)
{
  Options opts;
  ros::init(argc, argv, "tod_recognition");
  tod::RecognitionNode recognition_node(opts);
  ros::spin();
  return 0;
}
