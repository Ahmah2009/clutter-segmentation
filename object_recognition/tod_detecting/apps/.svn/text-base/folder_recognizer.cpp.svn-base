/*
 * main.cpp
 *
 *  Created on: Dec 18, 2010
 *      Author: Alexander Shishkov
 */
#include "tod/detecting/Loader.h"
#include "tod/detecting/Recognizer.h"

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#define foreach BOOST_FOREACH

using namespace cv;
using namespace tod;
using namespace std;
namespace po = boost::program_options;

namespace
{
struct Options
{
  std::string imageDirectory;
  std::string baseDirectory;
  std::string config;
  std::string logFilename;
  TODParameters params;
  int verbose;
  int mode;
};
}

int options(int ac, char ** av, Options& opts)
{

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("image,I", po::value<string>(&opts.imageDirectory), "Query image. Required.");
  desc.add_options()("base,B", po::value<string>(&opts.baseDirectory)->default_value("./"),
                     "The directory that the training base is in.");
  desc.add_options()("tod_config,f", po::value<string>(&opts.config), "The name of the configuration file");
  desc.add_options()("log,l", po::value<string>(&opts.logFilename), "The name of the log file");
  desc.add_options()("verbose,V", po::value<int>(&opts.verbose)->default_value(1), "Verbosity level");
  desc.add_options()("mode,m", po::value<int>(&opts.mode)->default_value(0), "Mode");

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
    cout << "Must supply an image directory." << "\n";
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
  }

  TrainingBase base(objects);
  Ptr<FeatureExtractor> extractor = FeatureExtractor::create(opts.params.feParams);

  FileStorage fs;
  fs.open(opts.logFilename, FileStorage::WRITE);
  fs << "trainFolder" << opts.baseDirectory;
  fs << "test1" << "{";
  fs << "testFolder" << opts.imageDirectory;
  fs << "objects" << "{";

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

  boost::filesystem::directory_iterator end_itr;
  int objectIndex = 1;
  for (boost::filesystem::directory_iterator itr(opts.imageDirectory); itr != end_itr; ++itr)
  {
    size_t position = itr->leaf().rfind(".png");
    if (position!=string::npos)
    {
      std::string image_name = itr->leaf();
      string path = opts.imageDirectory + "/" + image_name;
      cout << "< Reading the image... " << path;

      Features2d test;
      test.image = imread(path, 0);
      cout << ">" << endl;
      if (test.image.empty())
      {
        cout << "Can not read test image" << endl;
        break;
      }
      extractor->detectAndExtract(test);

      Mat drawImage;
      if (test.image.channels() > 1)
        test.image.copyTo(drawImage);
      else
        cvtColor(test.image, drawImage, CV_GRAY2BGR);

      vector<tod::ObjectInfo> foundObjects;
      recognizer->match(test, foundObjects);

      foreach(const ObjectInfo& object, foundObjects)
      {
        stringstream nodeIndex;
        nodeIndex << objectIndex++;
        string nodeName = "object" + nodeIndex.str();
        fs << nodeName << "{";
        fs << "id" << object.objectId;
        fs << "name" << object.objectName;
        fs << "rvec" << object.rvec;
        fs << "tvec" << object.tvec;
#if 0
        int index = atoi((image_name.substr(0, position)).c_str());
        fs << "imageIndex" << index;
#endif
        fs << "imageName" << image_name;
        fs << "idx" << object.imgIdx;
        fs << "}";
      }
    }
  }
  fs << "objectsCount" << objectIndex - 1;
  fs << "}" << "}";
  fs.release();
  return 0;
}
