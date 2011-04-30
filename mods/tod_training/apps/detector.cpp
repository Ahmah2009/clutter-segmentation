/*
 * detector.cpp
 *
 *  Created on: Nov 17, 2010
 *      Author: erublee
 */

#include <tod/core/stats.h>

#include "tod/training/feature_extraction.h"
#include "tod/training/masking.h"
#include "tod/training/file_io.h"
#include "fiducial/fiducial.h"
#include "tod/training/Opts.h"

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace std;
using namespace cv;

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH
#define sync(lock, code) { boost::mutex::scoped_lock sl(lock); code; }

namespace po = boost::program_options;

namespace
{
struct detector_options
{
  CommonOptions common;
  CameraOptions camera;
  ImagesOptions images;
  std::string feature_extractor_config;
  FeatureExtractionParams fe_params;
};

int options(int ac, char ** av, detector_options& opts)
{
  // Declare the supported options.
  po::options_description desc("detector options");
  desc.add(opts.common.desc);
  desc.add(opts.camera.desc);
  desc.add(opts.images.desc);
  desc.add_options()("feature_extractor_config,f",
                     po::value<string>(&opts.feature_extractor_config)->default_value("features.config.yaml"),
                     "The name of the feature extraction configuratoin file");
  desc.add_options()("generate_config",

                       "generate a config file");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("generate_config"))
  {
    FileStorage fs("features.config.yaml", FileStorage::WRITE);
    fs << FeatureExtractionParams::YAML_NODE_NAME;
    FeatureExtractionParams::CreateSampleParams().write(fs);
    cout << "See newly generated features.config.yaml" << endl;
    return 1;
  }
  try
  {
    opts.common.check(vm);
    FileStorage fs;
    if (opts.feature_extractor_config.empty()
        || !(fs = FileStorage(opts.feature_extractor_config, FileStorage::READ)).isOpened())
    {
      FileStorage fs("features.config.yaml", FileStorage::WRITE);
      fs << FeatureExtractionParams::YAML_NODE_NAME;
      FeatureExtractionParams::CreateSampleParams().write(fs);
      throw std::runtime_error("Must supply feature extraction configuration. see newly generated features.config.yaml");
    }
    else
      opts.fe_params.read(fs[FeatureExtractionParams::YAML_NODE_NAME]);

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
  features_worker(const std::string& directory, const list<string>& images, FeatureExtractionParams fe_params,
                  Camera camera,bool verbose, detector_stats & stats) :
    directory(directory), images(images), fe_params(fe_params), camera(camera),verbose(verbose), stats(stats)
  {
  }

  void operator()()
  {

    Ptr<FeatureExtractor> extractor(FeatureExtractor::create(fe_params));
    if (extractor.empty())
      throw std::runtime_error("bad FeatureExtractorParams!");
    Mat features_draw;

    //given an image and mask detect and extract
    foreach(string x,images)
          {
            sync(stats_lock, stats.img_cnt++);

            string maskfile = filename(x + ".mask.png");
            FileMasker masker(maskfile);

            Mat gray = imread(filename(x), CV_LOAD_IMAGE_GRAYSCALE);

            Features2d f2d(camera, gray);
            f2d.image_name = x;

            masker.mask(f2d);
            if (f2d.mask.empty())
            {
              std::cout << "[DETECTOR] FAILED to detect features (no mask): " << x << std::endl;
            sync(stats_lock, stats.failure_cnt++);
             // mout("\nno mask for file: " << x << endl);
              continue;
            }
            f2d.mask_name = x + ".mask.png";

            fiducial::KnownPoseEstimator pose_est(filename(x + ".pose.yaml")); //load pose from file
            f2d.camera.pose = pose_est.estimatePose(Mat());

            std::string features_file = filename(x + ".features.yaml.gz");

            extractor->detectAndExtract(f2d);
            sync(stats_lock, stats.success_cnt++);

            if(verbose){
              boost::mutex::scoped_lock sl(imshow_lock);
              f2d.draw(features_draw, 0);
              imshow("features", features_draw);
              // imshow("mask",f2d.mask);
              waitKey(0);
            }

            FileStorage fs(features_file, FileStorage::WRITE);
            fs << FeatureExtractionParams::YAML_NODE_NAME;
            fe_params.write(fs);
            fs << Features2d::YAML_NODE_NAME;
            f2d.write(fs);
          //  mout("." << flush);
          }
  }

  static boost::mutex cout_lock;
  static boost::mutex stats_lock;
  static boost::mutex imshow_lock;
  string directory;
  list<string> images;
  FeatureExtractionParams fe_params;
  Camera camera;
  bool verbose;
  detector_stats & stats;

};

boost::mutex features_worker::cout_lock;
boost::mutex features_worker::stats_lock;
boost::mutex features_worker::imshow_lock;
int main(int argc, char *argv[])
{
  detector_options opts;
  if (options(argc, argv, opts))
    return 1;

  list<string> images = opts.images.loadImageNames(opts.common);
  Camera camera = opts.camera.loadCamera(opts.common);

  //first split the image list into the number of threads that we want
  std::vector<std::list<std::string> > vlist = splitList(images, opts.common.n_threads);

  detector_stats stats;
  clock_t before = clock();

  //create a thread for each sublist
  boost::thread_group threads;
  foreach(const std::list<std::string>& x,vlist)
        {
          threads.create_thread(features_worker(opts.common.directory, x, opts.fe_params, camera,opts.common.verbose, stats));
        }

  //join all the threads that we spawned
  threads.join_all();
    
  clock_t after = clock();
  stats.time = float (after - before) / CLOCKS_PER_SEC;
  cout << stats;

  ofstream stats_out((opts.common.directory + "/detector_stats.yaml").c_str());
  stats_out << stats;
  stats_out.close();


  return 0;
}
