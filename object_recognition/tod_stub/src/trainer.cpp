#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>

#include <tod_stub/tod_stub.h>
#include <tod_stub/tod_stub_impl.h>

#include <tod/training/ros/msgs.h>
#include <tod/training/pose.h>
#include <tod/training/Opts.h>

#include <opencv2/calib3d/calib3d.hpp>

using std::cout;
using std::endl;
using namespace tod_stub;
namespace
{
struct TrainingProcessor
{
  boost::shared_ptr<Trainer> trainer;
  TrainingProcessor(Options opts,const std::string& fiducial, const std::string& object_id) :
    trainer(createTrainer(opts.config_file)), fiducial_yml(fiducial), object_id(object_id)
  {

  }
  void operator()(const ImagePointsCamera& ipc)
  {
    if (!posestimator)
    {
      tod::Camera camera = tod::fromRosMsg(ipc.camera_info);
      cv::FileStorage fs(fiducial_yml, cv::FileStorage::READ);
      tod::Fiducial fiducial;
      fiducial.read(fs["fiducial"]);
      posestimator.reset(new tod::FiducialPoseEstimator(fiducial, camera, false));
    }
    ipc.convert(train_data.frame);
    train_data.object_id = object_id;
    tod::PoseRT pose = posestimator->estimatePose(train_data.frame.image);
    if (pose.estimated)
    {
      cv::Rodrigues(pose.rvec, train_data.R);
      train_data.T = pose.tvec;
    }
    trainer->process(train_data);
  }
  TrainData train_data;
  std::string fiducial_yml;
  std::string object_id;
  boost::shared_ptr<tod::FiducialPoseEstimator> posestimator;
};
}

namespace po = boost::program_options;
int main(int argc, char ** argv)
{
  std::string fiducial_yaml;
  std::string object_id;
  Options opts;
  opts.desc.add_options()("fiducial,F", po::value<std::string>(&fiducial_yaml)->default_value("fiducial.yml"),
                          "The yaml file describing the fiducial marker, use gen_fiducial to see a sample yml file based on art/board.02.svg.");
  opts.desc.add_options()("object_id,I", po::value<std::string>(&object_id)->default_value("object_001"),
                          "The string id of the object being trained. This should be associated and used when ever refering to this object.");
  if (options(argc, argv, opts))
    return 1;

  boost::filesystem::path fy = fiducial_yaml;
  if (!boost::filesystem::exists(fy))
  {
    std::cerr << "Please specify an existing fiducial yaml file" << std::endl;
    std::cout << opts.desc << std::endl;
    return 1;
  }
  
  rosbag::Bag bag;
  bag.open(opts.bag_file, rosbag::bagmode::Read);
  TrainingProcessor p(opts, fiducial_yaml, object_id);
  p.trainer->onInit();
  ImagePointsCamera::ProcessBag(bag, boost::ref(p));
  p.trainer->onFinish();

  return 0;
}
