#include <tod/training/Opts.h>
#include <iostream>
#include <list>
#include <fstream>
#include <tod/training/file_io.h>
using namespace std;
namespace tod
{
namespace po = boost::program_options;

CommonOptions::CommonOptions() :
  desc("Common options"),verbose(false)
{
  // Declare the supported options.
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("directory,d", po::value<string>(&directory),
                     "The directory that the images are in. Also used for storing the pose estimations. Required.");

  desc.add_options()("threads,j", po::value<int>(&n_threads)->default_value(1),
                     "The number of threads to use during pos estimation, think jobs.");
  desc.add_options()("verbose,V",po::value<bool>(&verbose), "If verbose will display gui stuff.");

}

void CommonOptions::check(const boost::program_options::variables_map& vm) const throw (std::runtime_error)
{
  if (vm.count("help"))
  {
    return throw runtime_error("help");
  }
  if (!vm.count("directory"))
  {
    return throw runtime_error("must supply a directory");
  }
}

CameraOptions::CameraOptions() :
  desc("Camera")
{
  desc.add_options()("camera,K", po::value<string>(&camera_file), "The camera calibration file.");
  desc.add_options()("camera_format", po::value<int>(&camera_format)->default_value(Camera::TOD_YAML),
                     "The file format 0 for opencv format, 1 for tod format.");
}

Camera CameraOptions::loadCamera(const CommonOptions& opts) const
{
  Camera camera;
  if (camera_file.empty())
  {
    camera = Camera(opts.directory + "/camera.yml", (Camera::CalibrationFormat)camera_format);
  }
  else
  {
    camera = Camera(camera_file, (Camera::CalibrationFormat)camera_format);
  }
  return camera;
}

void CameraOptions::check(const boost::program_options::variables_map& vm) const throw (std::runtime_error)
{
  if (!vm.count("camera"))
  {
    return throw runtime_error("must supply a camera");
  }
}
ImagesOptions::ImagesOptions() :
  desc("Images")
{
  desc.add_options()("extension,E", po::value<string>(&extension)->default_value(".png"),
                     "The extension used for finding files.");
  desc.add_options()("images,I", po::value<string>(&image_list), "The list of images, use ls *.png > images.txt. "
    "Optional. by default will look for all pngs in the specified directory. Will be opened relative to directory.");

}
std::list<std::string> ImagesOptions::loadImageNames(const CommonOptions& opts) const
{
  std::list<std::string> images;
  if (image_list.empty())
    images = getFileList(opts.directory, extension);//getImageList(fin);
  else
  {
    ifstream fin((opts.directory + "/" + image_list).c_str());
    images = getImageList(fin);
  }
  if (images.empty())
  {
    throw std::runtime_error("could not read images list.  Try creating the list with:\nls *.png > images.txt");
  }
  return images;
}

PCDOptions::PCDOptions() :
  desc("Point Clouds")
{
  desc.add_options()("pcds,P", po::value<string>(&pcd_list),
                     "The list of point cloud files, use ls *.pcd > pcds.txt");
}
std::list<std::string> PCDOptions::loadPCDNames(const CommonOptions& opts) const
{
  std::list<std::string> pcds;
  if (pcd_list.empty())
    pcds = getFileList(opts.directory, ".pcd");//getImageList(fin);
  else
  {
    ifstream fin((opts.directory + "/" + pcd_list).c_str());
    pcds = getImageList(fin);

  }
  if (pcds.empty())
  {
       throw std::runtime_error("could not read pcd list.  Try creating the list with:\nls *.pcd > pcds.txt");
  }
  return pcds;
}

}
