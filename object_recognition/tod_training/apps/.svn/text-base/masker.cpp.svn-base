/*
 * masker.cpp
 *
 *  Created on: Dec 9, 2010
 *      Author: erublee
 */

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <tod/training/pose.h>
#include <tod/training/file_io.h>
#include <tod/training/Opts.h>
#include <tod/training/masking.h>

#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/registration/registration.h"

#include "tod/core/PoseRT.h"
#include "tod/core/Camera.h"
#include "tod/training/clouds.h"

#include <tod/core/Features3d.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace std;
using namespace cv;

namespace po = boost::program_options;

namespace
{

struct masker_options
{
  CommonOptions common;
  CameraOptions camera;
  ImagesOptions images;
  PCDOptions pcds;
  int mask_type;
};

int options(int ac, char ** av, masker_options& opts)
{
  // Declare the supported options.
  po::options_description desc("masker options");
  desc.add(opts.common.desc);
  desc.add(opts.camera.desc);
  desc.add(opts.images.desc);
  desc.add(opts.pcds.desc);
  desc.add_options()("mask_type,M", po::value<int>(&opts.mask_type)->default_value(0),
      "The mask type, 0 for 3d box, 1 for pcl table segmentation");

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

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

#define filename(arg) (directory+"/"+arg)

#define mout(x) {boost::mutex::scoped_lock sl(cout_lock);cout << x;}

struct masker_worker
{
  typedef pcl::PointXYZ point_t;
  typedef pcl::PointCloud<point_t> cloud_t;
  masker_worker(const std::string& directory, const Camera& camera, const list<string>& images,
                const list<string>& pcds, const Masker* masker, const masker_options& opts) :
                  directory(directory), images(images), pcds(pcds), masker(masker), camera(camera), opts(opts)
  {

  }
  void operator()()
  {
    if (!pcds.empty() && pcds.size() != images.size())
    {
      throw std::runtime_error("pcd_list and image_list need to be the same size!");
    }

    list<string>::const_iterator pcd_name = pcds.begin();

    foreach( const std::string& x, images )
    {
      try
      {
        KnownPoseEstimator pose_est(filename(x + ".pose.yaml")); //load pose from file
        cv::Mat color = imread(filename(x), CV_LOAD_IMAGE_COLOR);
        Features2d f2d(camera, color);
        f2d.camera.pose = pose_est.estimatePose(Mat());
        if (!pcds.empty()) //TODO make better hacked ugly for now
        {
          cloud_t pcl_cloud;
          pcl::io::loadPCDFile(filename(*pcd_name), pcl_cloud);
          Mat mask = cloudMask(pcl_cloud, f2d.camera.pose, camera);
          f2d.mask = mask;
        }
        else
        {
          masker->mask(f2d);
        }

        if (!f2d.mask.empty())
        {
          if (opts.common.verbose)
          {
            boost::mutex::scoped_lock sl(showmask_lock);
            cv::Mat colorMask;
            cvtColor(f2d.mask, colorMask, CV_GRAY2BGR);
            imshow("mask", f2d.image & colorMask);
            //imshow("mask", f2d.mask);
            waitKey(30);
          }
          //append the mask name to a
          string mask_name = x + ".mask.png";
          imwrite(filename(mask_name), f2d.mask);
          //mout("." << flush)
        }

      }
      catch (std::runtime_error)
      {
        //              /mout( "\nno known pose for item: " << x << endl)
      }

      if (!pcds.empty())
      {
        ++pcd_name;
      }

    }

  }

  static boost::mutex cout_lock;
  static boost::mutex showmask_lock;
  string directory;
  list<string> images;
  list<string> pcds;
  const Masker* masker;
  string outputfile;
  Camera camera;
  masker_options opts;
};

boost::mutex masker_worker::cout_lock;
boost::mutex masker_worker::showmask_lock;

int main(int argc, char *argv[])
{
  masker_options opts;
  if (options(argc, argv, opts))
    return 1;

  //read in the image list provided on the command line
  list<string> images = opts.images.loadImageNames(opts.common);
  list<string> pcds;

  if (images.empty())
  {
    return 1;
  }

  Camera camera = opts.camera.loadCamera(opts.common);
  Ptr<Masker> masker;
  switch (opts.mask_type)
  {
    case 0:
    {
      Select3dObjecMasker* s3dobj = new Select3dObjecMasker(camera);
      masker = s3dobj;
      foreach( const std::string& x, images )
      {
        try
        {
          KnownPoseEstimator pose_est(opts.common.directory + "/" + x + ".pose.yaml");
          cv::Mat color = imread(opts.common.directory + "/" + x);
          Features2d f2d(camera, color);
          f2d.camera.pose = pose_est.estimatePose(Mat());
          if (s3dobj->getBox().empty())
          {
            if (!s3dobj->guiSelectBox(f2d))
            {
              cout << "quiting" << endl;
              return 0;
            }
          }
          else
            break; //the box was selected
        }
        catch (std::runtime_error)
        {
          cout << "no known pose for item: " << x << endl;
        }
      }
    }
    break;
    case 1:
      pcds = opts.pcds.loadPCDNames(opts.common);
      if (pcds.empty())
      {

        return 1;
      }
      if (pcds.size() != images.size())
      {
        throw std::runtime_error("must have the same number of point clouds as images");
      }
      // throw std::runtime_error("pcl table segmenter not implemented :(");
      break;
    default:
      cerr << "bad mask type" << endl;
      return 1;

  }

  //spawn some threads so that we get this done in a reasonable amount of time

  //first split the image list into the number of threads that we want
  std::vector<std::list<std::string> > ilist = splitList(images, opts.common.n_threads);

  std::vector<std::list<std::string> > plist;
  if (pcds.size())
    plist = splitList(pcds, opts.common.n_threads);

  //create a thread for each sublist
  boost::thread_group threads;
  for (size_t i = 0; i < ilist.size(); i++)
  {
    {
      threads.create_thread(masker_worker(opts.common.directory, camera, ilist[i], pcds.empty() ? pcds : plist[i],
          &*masker, opts));
    }
  }
  //join all the threads that we spawned
  threads.join_all();
  cout << "\n";

  return 0;
}
