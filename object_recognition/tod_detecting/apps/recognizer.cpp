/*
 * main.cpp
 *
 *  Created on: Dec 18, 2010
 *      Author: alex
 */
#include "tod/detecting/Loader.h"
#include "tod/detecting/Parameters.h"
#include "tod/detecting/Recognizer.h"
#include "tod/training/feature_extraction.h"

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#define foreach BOOST_FOREACH
typedef std::pair<int, int > idx_pair_t;
using std::list;
using std::string;
using std::istream;
using namespace cv;
using namespace tod;

namespace po = boost::program_options;

namespace
{
struct Options
{
  std::string imageFile;
  std::string baseDirectory;
  std::string config;
  TODParameters params;
  int verbose;
  int mode;
};
}

void drawProjections(const Mat& image, int id, const vector<Guess>& guesses, const TrainingBase& base,
                     const Options& opts)
{
  if (guesses.empty())
    return;
  Mat drawImg, correspondence;
  image.copyTo(drawImg);
  namedWindow("proj", 1);
  namedWindow("correspondence", CV_WINDOW_KEEPRATIO);
  foreach(const Guess& guess, guesses)
  {
    guess.draw(drawImg, 0, ".");
    Mat temp, small;
    drawImg.copyTo(temp);
    resize(temp, small, Size(640, 480), CV_INTER_LINEAR);
    small.copyTo(drawImg);
    imshow("proj", drawImg);
    guess.draw(correspondence, 1, opts.baseDirectory);
    if (!correspondence.empty())
      imshow("correspondence", correspondence);
    waitKey(0);
  }
}

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("image,I", po::value<string>(&opts.imageFile), "Query image. Required.");
  desc.add_options()("base,B", po::value<string>(&opts.baseDirectory)->default_value("./"),
      "The directory that the training base is in.");
  desc.add_options()("tod_config,f", po::value<string>(&opts.config), "The name of the configuration file");
  desc.add_options()("verbose,V", po::value<int>(&opts.verbose)->default_value(1), "Verbosity level");
  desc.add_options()("mode,m", po::value<int>(&opts.mode)->default_value(0),
      "Mode: 0-textured object detection, 1 - kinect version");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  FileStorage fs;
  if (opts.config.empty() || !(fs = FileStorage(opts.config, FileStorage::READ)).isOpened())
  {
    cout << "Must supply configuration. see newly generated sample.config.yaml" << "\n";
    cout << desc << endl;
    FileStorage fs("./sample.config.yaml", FileStorage::WRITE);
    fs << TODParameters::YAML_NODE_NAME;
    TODParameters::CreateSampleParams().write(fs);
    return 1;
  }
  else
    opts.params.read(fs[TODParameters::YAML_NODE_NAME]);

  if (!vm.count("image"))
  {
    cout << "Must supply an image file." << "\n";
    cout << desc << endl;
    return 1;
  }

  if (!vm.count("base"))
  {
    cout << "Must supply training base directory." << "\n";
    cout << desc << endl;
    return 1;
  }

  return 0;
}

//This function takes one image given by on the command line.
//Reads in a given training set, extracts features from the image and
//brute force matches it against the training base using ransac etc
// Efficiency could be improved by voting on the image for likely object(s) and only matching against those
int main(int argc, char* argv[])
{
  Options opts;
  if (options(argc, argv, opts))
    return 1;

  tod::Loader loader(opts.baseDirectory);
  vector<cv::Ptr<TexturedObject> > objects;
  loader.readTexturedObjects(objects);

  if (!objects.size())
  {
    cout << "Empty base\n" << endl;
    return 1;
  }

  TrainingBase base(objects);

  Ptr<FeatureExtractor> extractor = FeatureExtractor::create(opts.params.feParams);
  Features2d test;
  test.image = imread(opts.imageFile, 0);
  extractor->detectAndExtract(test);
  cout << "Extracted " << test.keypoints.size() << " points" << endl;

  cv::Ptr<Matcher> rtMatcher = Matcher::create(opts.params.matcherParams);
  rtMatcher->add(base);

  cv::Ptr<Recognizer> recognizer;
  if (opts.mode == TOD)
  {
    recognizer = new TODRecognizer(&base, rtMatcher, &opts.params.guessParams,
                                   opts.verbose, opts.baseDirectory, opts.params.clusterParams.maxDistance);
  }
  else if (opts.mode == KINECT)
  {
    recognizer = new KinectRecognizer(&base, rtMatcher, &opts.params.guessParams, opts.verbose, opts.baseDirectory);
  }
  else
  {
    cout << "Invalid mode option!" << endl;
    return 1;
  }

  vector<tod::ObjectInfo> foundObjects;
  recognizer->match(test, foundObjects);

  foreach(const ObjectInfo& object, foundObjects)
  {
    cout << "Object name = " << object.objectName << ", imageId = " << object.imgIdx << endl;
  }

  return 0;
}
