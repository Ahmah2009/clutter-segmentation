/*
 * detector.cpp
 *
 *  Created on: Nov 17, 2010
 *      Author: erublee
 */
#include "tod/training/feature_extraction.h"
#include "tod/training/masking.h"
#include "tod/training/file_io.h"
#include "tod/training/clouds.h"
#include "tod/training/Opts.h"

#include "fiducial/fiducial.h"

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <ctime>
#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace std;
using namespace cv;

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace po = boost::program_options;

namespace
{

struct f3d_options
{
  CommonOptions common;
  CameraOptions camera;
  ImagesOptions images;
  PCDOptions pcds;
  int mask_type;
};

int options(int ac, char ** av, f3d_options& opts)
{
  // Declare the supported options.
  po::options_description desc("f3d_creator options");
  desc.add(opts.common.desc);
  desc.add(opts.camera.desc);
  desc.add(opts.images.desc);
  desc.add(opts.pcds.desc);

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);
  try
  {
    opts.common.check(vm);
    //opts.camera.check(vm);
  }
  catch (std::runtime_error e)
  {
    cout << desc << endl;
    cerr << e.what() << endl;
    return 1;
  }
  return 0;
}
}

#define filename(arg) (directory+"/"+arg)
#define mout(x) {boost::mutex::scoped_lock sl(cout_lock);cout << x;}

struct features_worker
{
  features_worker(const std::string& directory, const list<string>& images, const list<string>& pcds, Camera camera) :
    directory(directory), images(images), pcds(pcds), camera(camera), mapper(new CameraProjector(camera))
  {
  }

  void operator()()
  {
    if (images.size() != pcds.size())
      throw std::runtime_error("pcd_list and image_list need to be the same size!");

    list<string>::const_iterator image_name = images.begin(), pcd_name = pcds.begin();
    //given an image and mask detect and extract
    for (; image_name != images.end(); ++image_name, ++pcd_name)
    {
      //mout(filename(*pcd_name) << " " << filename(*image_name) << endl);

      string features_file = filename(*image_name + ".features.yaml.gz");
      FileStorage fs(features_file, FileStorage::READ);
      if (!fs.isOpened())
        continue;

    // The heavy load on both CPU and IO often makes the operating
    // system very unresponsive, so take a little break.
    timespec t;
    t.tv_sec = 0;
    t.tv_nsec = int(1e8);
    nanosleep(&t, NULL);


      Features2d f2d;
      f2d.read(fs[Features2d::YAML_NODE_NAME]);
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      // mout(filename(*pcd_name) << endl);
      // continue;
      pcl::io::loadPCDFile(filename(*pcd_name), pcl_cloud);
      Cloud cloud;
      Mat pcl_image;
      // Cloud2Cv(pcl_cloud,pcl_image);

      //todo use point cloud .at(u,v)...
      //todo verify that this works!
      tod::PCLToPoints(cloud, pcl_cloud, f2d);

      if (pcl_cloud.is_dense == false)
        mapper.map(f2d, cloud, cloud);

      filterCloudNan(cloud, f2d);

      string f3d_file = filename(*image_name + ".f3d.yaml.gz");
      Features3d f3d(f2d, cloud);

      FileStorage fs3d(f3d_file, FileStorage::WRITE);
      fs3d << Features3d::YAML_NODE_NAME;
      f3d.write(fs3d);
    }
  }

  static boost::mutex cout_lock;
  static boost::mutex imshow_lock;
  string directory;
  list<string> images, pcds;
  Camera camera;
  ProjectiveAppoximateMapper mapper;

};

boost::mutex features_worker::cout_lock;
boost::mutex features_worker::imshow_lock;
int main(int argc, char *argv[])
{
  f3d_options opts;
  if (options(argc, argv, opts))
    return 1;

  //read in the image list provided on the command line
  list<string> images = opts.images.loadImageNames(opts.common);
  list<string> pcds = opts.pcds.loadPCDNames(opts.common);
  Camera camera = opts.camera.loadCamera(opts.common);

  //spawn some threads so that we get this done in a reasonable amount of time

  //first split the image list into the number of threads that we want
  std::vector<std::list<std::string> > ilist = splitList(images, opts.common.n_threads);
  std::vector<std::list<std::string> > plist = splitList(pcds, opts.common.n_threads);

  if (ilist.size() != plist.size())
    throw std::runtime_error("pcd list and image list must be the same size!");

  //create a thread for each sublist
  boost::thread_group threads;

  for (size_t i = 0; i < ilist.size(); i++)
  {
    threads.create_thread(features_worker(opts.common.directory, ilist[i], plist[i], camera));
  }

  //join all the threads that we spawned
  threads.join_all();

  return 0;
}
