/*
 * cluster_sample.cpp
 *
 *  Created on: Jan 4, 2011
 *      Author: Ethan Rublee, original Alexander
 */
#include "tod/detecting/Loader.h"
#include "tod/detecting/clustering.h"
#include "tod/detecting/Cluster.h"
#include "tod/detecting/Matcher.h"
#include "tod/detecting/GuessGenerator.h"
#include "tod/detecting/Parameters.h"
#include "tod/training/feature_extraction.h"
#include "tod/core/TexturedObject.h"
#include "posest/pnp_ransac.h"
#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#define foreach BOOST_FOREACH

using std::list;
using std::string;
using std::istream;
using namespace cv;
using namespace tod;

namespace po = boost::program_options;

namespace
{
struct Options{
  std::string camera_file;
  std::string image_file;
  std::string params_file;
  Mat test_image;
  TODParameters params;
  Camera camera;
};
}

void drawProjections(const Mat& image, int id, const vector<Guess>& guesses, const TrainingBase& base)
{
  Mat drawImg;
  image.copyTo(drawImg);
  namedWindow("proj", 1);

  foreach(const Guess& guess, guesses)
  {
    vector<Point2f> projectedPoints;
    projectPoints(Mat(base.getObject(id)->observations[guess.imageIndex].cloud()), guess.pose().rvec, guess.pose().tvec,
                  base.getObject(id)->observations[guess.imageIndex].camera().K,
                  base.getObject(id)->observations[guess.imageIndex].camera().D, projectedPoints);

    for (size_t m = 0; m < projectedPoints.size(); m++)
    {
      circle(drawImg, projectedPoints[m], 3, Scalar(255, 0, 0), 1);
    }
  }
  imshow("proj", drawImg);
  waitKey(0);
}

int options(int ac, char ** av,Options& opts){

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
  ("help", "Produce help message.")
  ("camera,K",po::value<string>(&opts.camera_file), "Camera calibration file - opencv yaml format")
  ("image,I",po::value<string>(&opts.image_file), "Query image. Required.")
  ("tod_config,f",po::value<string>(&opts.params_file), "The name of the configuration file")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
      cout << desc << "\n";
      return 1;
  }
  //load tod configuration
  {
    FileStorage fs;
    if(opts.params_file.empty() || ! (fs = FileStorage(opts.params_file, FileStorage::READ)).isOpened())
    {
        cout << "Must supply configuration. see newly generated sample.config.yaml" << "\n";
        cout << desc << endl;
        FileStorage fs("./sample.config.yaml",FileStorage::WRITE);
        fs << TODParameters::YAML_NODE_NAME;
        TODParameters::CreateSampleParams().write(fs);
        return 1;
    }else
      opts.params.read(fs[TODParameters::YAML_NODE_NAME]);
  }

  if (!vm.count("image"))
  {
    cout << "Must supply an image file." << "\n";
    cout << desc << endl;
    return 1;
  }else
  {
    opts.test_image = imread(opts.image_file,CV_LOAD_IMAGE_GRAYSCALE);
    if(opts.test_image.empty()){
      cout << "error opening image file: "<< opts.image_file << "\n";
         return 1;
    }
  }

  if (!vm.count("camera"))
  {
    cout << "Must supply a camera calibration file." << "\n";
    cout << desc << endl;
    return 1;
  }

  opts.camera = Camera(opts.camera_file, Camera::OPENCV_YAML);

  return 0;

}

void cluster( Ptr<Clusterer> clusterer, const Features3d& f3d ){
   double t = (double)getTickCount();
   vector<Features3d> clusters;
   clusterer->cluster(f3d,clusters);

   double total_t = ((double)getTickCount() - t )/getTickFrequency();
   Mat cluster_img;
   Clusterer::DrawClusters(clusters,cluster_img);


   imshow("clusters",cluster_img);

   cout << "Found " << clusters.size() << " clusters. Took " << total_t * 1000 << " milliseconds" <<  endl;
}


bool changed = true;

void onIterationsChange(int position, void* _data)
{
  MeanShiftParams* params = reinterpret_cast<MeanShiftParams*> (_data);
  if (!params)
    return;
  params->iterations = position;
  changed = true;
}

void onEpsChange(int position, void* _data)
{
  MeanShiftParams* params = reinterpret_cast<MeanShiftParams*> (_data);
  if (!params)
    return;
  params->epsilon = position * 1./10;
  changed = true;
}

void onLamdaChange(int position, void* _data)
{
  MeanShiftParams* params = reinterpret_cast<MeanShiftParams*> (_data);
  if (!params)
    return;
  params->lambda = position;
  changed = true;
}

void runExample(Features3d& f3d)
{
  std::string matches_window = "clusters";
  namedWindow(matches_window, CV_WINDOW_KEEPRATIO);
  int i = 10, e = 40, l = 100;
  MeanShiftParams ms_params(i, l, e / 10., false);
  createTrackbar("iterations", matches_window, &i, 200, onIterationsChange, &ms_params);
  createTrackbar("epsilon (1/10)", matches_window, &e, 1000, onEpsChange, &ms_params);
  createTrackbar("lamda", matches_window, &l, 500, onLamdaChange, &ms_params);
  char key = 0;
  cout << "press q or Esc to quit" << endl;
  while (key != 'q' && key != 27)
  {
    if (changed)
    {
      changed = false;
      Ptr<Clusterer> clusterer(new MeanShiftClustering(ms_params));
      cluster(clusterer, f3d);

    }
    key = waitKey(100);
  }
  std::string params_file = "sample.ms_params.yaml";
  FileStorage fs(params_file,FileStorage::WRITE);
  fs << "clusterer";
  ms_params.write(fs);
  cout << "saving last params to " << params_file << endl;
}

int main(int argc, char* argv[])
{
  Options opts;
  if (options(argc, argv, opts))
    return 1;

  Ptr<FeatureExtractor> extractor = FeatureExtractor::create(opts.params.feParams);

  Features2d test(opts.camera,opts.test_image);
  extractor->detectAndExtract(test);

  Features3d test3d(test);
  runExample(test3d);

  return 0;
}
