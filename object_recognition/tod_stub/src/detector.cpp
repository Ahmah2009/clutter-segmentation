#include <boost/foreach.hpp>

#include <iostream>
#include <string>

#include <tod_stub/tod_stub.h>
#include <tod_stub/tod_stub_impl.h>

#include <pcl_visualization/cloud_viewer.h>
using std::cout;
using std::endl;
using namespace tod_stub;
namespace
{
struct DetectingProcessor
{
  DetectingProcessor(Options opts) :
    detector(createDetector(opts.config_file)), cloud_viewer("Results")
  {
    cloud_viewer.runOnVisualizationThread(boost::ref(pose_drawer));
  }
  void operator()(const ImagePointsCamera& ipc)
  {

    ipc.convert(frame_data);
    std::vector<Result> results;
    detector->detect(frame_data, results);
    cloud_viewer.showCloud(frame_data.cloud,"cloud");
    pose_drawer.drawResults(results);
  }
  boost::shared_ptr<Detector> detector;
  FrameData frame_data;
  pcl_visualization::CloudViewer cloud_viewer;
  PoseDrawer3d pose_drawer;
};
}

int main(int argc, char ** argv)
{
  Options opts;
  if (options(argc, argv, opts))
    return 1;

  rosbag::Bag bag;
  bag.open(opts.bag_file, rosbag::bagmode::Read);
  DetectingProcessor p(opts);

  ImagePointsCamera::ProcessBag(bag, boost::ref(p));

  bag.close();
}
