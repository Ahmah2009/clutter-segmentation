/*
 * Loader.cpp
 *
 *  Created on: Dec 15, 2010
 *      Author: alex
 */
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <fstream>
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <tod/detecting/Loader.h>
#include <tod/training/file_io.h>
#include <tod/training/clouds.h>
#include <tod/detecting/Tools.h>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "tod/training/pose.h"
#include "tod/core/PoseRT.h"

#include <opencv2/core/eigen.hpp>

#define filename(arg) (directory+"/"+arg)
#define foreach BOOST_FOREACH

using std::list;
using std::string;
using std::istream;
using namespace cv;

namespace tod
{
std::string Loader::CONFIG_NAME = "config.txt";
std::string Loader::IMAGES_LIST = "images.txt";
std::string Loader::CLOUDS_LIST = "pcds.txt";
std::string Loader::POSTFIX = ".f3d.yaml.gz";
static std::string IMAGES_LIST;
Loader::Loader(const std::string& folderName)
{
  directory = folderName;
}

list<string> getStringsList(istream& input)
{
  list<string> imlist;
  while (!input.eof() && input.good())
  {
    string imname;
    input >> imname;
    if (!imname.empty())
      imlist.push_back(imname);
  }
  return imlist;
}

void Loader::findCloudFile(string& cloudFile, const string& objectPath)
{
  ifstream pcdsFile;
  pcdsFile.open((objectPath + "/" + CLOUDS_LIST).c_str());
  if (pcdsFile.is_open())
  {
      pcdsFile >> cloudFile;
      pcdsFile.close();
  }
  else
  {
    list<string> cloudNames = getFileList(objectPath, ".pcd");
    if (cloudNames.size())
      cloudFile = *cloudNames.begin();
  }
  cloudFile = objectPath + "/" + cloudFile;
}

void Loader::calculateStddev(Ptr<TexturedObject>& object, const string& objectPath)
{
  string cloudFile;
  findCloudFile(cloudFile, objectPath);
  if (!cloudFile.empty())
  {
    vector<Point3f> cloud;
    if (cloudFile.find(".txt") != std::string::npos)
    {
      ifstream txtfile;
      txtfile.open(cloudFile.c_str());
      while (!txtfile.eof() && txtfile.good())
      {
        Point3f p;
        txtfile >> p.x >> p.y >> p.z;
        cloud.push_back(p);
      }
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::io::loadPCDFile(cloudFile, pcl_cloud);

      for (size_t ind = 0; ind < pcl_cloud.points.size(); ind++)
      {
        pcl::PointXYZRGB& p = pcl_cloud.points[ind];
        cloud.push_back(Point3f(p.data[0], p.data[1], p.data[2]));
      }
      filterCloudNan(cloud);
    }
    if (cloud.size())
    {
      object->stddev = Tools::computeStdDev(cloud);
    }
  }
}

void Loader::calculateStddev(Ptr<TexturedObject>& object)
{
  float stddev = -1.0;
  for (size_t obsIndex = 0; obsIndex < 1; obsIndex++)
  {
    const vector<Point3f>& cloud = object->observations[obsIndex].cloud();
    if (cloud.size())
    {
      float curStdDev = Tools::computeStdDev(cloud);
      if (curStdDev > stddev)
        stddev = curStdDev;
    }
  }
  object->stddev = stddev;
}

void Loader::readTexturedObjects(vector<Ptr<TexturedObject> >& objects)
{
  objects.clear();

  ifstream fin(filename(CONFIG_NAME).c_str());
  list<string> objectNames = getStringsList(fin);
  fin.close();

  foreach(const string& name, objectNames)
  {
    Ptr<TexturedObject> object = new TexturedObject();
    object->name = name;

    // Get the list of files to read
    ifstream imagesFile(filename(name + "/" + IMAGES_LIST).c_str());
    list<string> imageNames = getStringsList(imagesFile);
    imagesFile.close();

    // If that list is empty, figure it out automatically by looking at the camera files and sampling them smartly
          if (imageNames.empty())
          {
            // Go over each camera file in the folder
            boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
            typedef std::pair<float, std::string> AngleImage;
            std::vector<AngleImage> angle_images;

            Eigen::Matrix<float, 3, 3> first_r;
            Eigen::Vector3f first_t;

            bool is_first_pose = true;
            for (boost::filesystem::directory_iterator itr(directory + "/" + name); itr != end_itr; ++itr)
            {
              // Check out the camera file
              size_t position = itr->leaf().rfind(".pose.yaml");
              if (position!=string::npos)
              {
                // get the angle of the corresponding image
                std::string image_name = itr->leaf().substr(0, position);
                tod::PoseRT pose =
                    tod::KnownPoseEstimator((directory + "/" + name + "/" + itr->leaf())).estimatePose(cv::Mat());

                // Store the first pose aside
                if (is_first_pose)
                {
                  cv::Mat_<float> rotation(3, 3);
                  cv::Rodrigues(pose.rvec, rotation);
                  cv::cv2eigen(rotation, first_r);
                  cv::cv2eigen(pose.tvec, first_t);
                  is_first_pose = false;
                }

                // Figure out the position of (1,0,0) in the first frame
                // First, figure out the position of (1,0,0) in the camera frame
                Eigen::Matrix<float, 3, 3> r;
                Eigen::Vector3f t, point;
                cv::Mat rotation;
                cv::Rodrigues(pose.rvec, rotation);
                cv::cv2eigen(rotation, r);
                cv::cv2eigen(pose.tvec, t);
                // Compute the image of (1,0,0) in the camera frame
                point[0] = 1;
                point[1] = 0;
                point[2] = 0;
                point = r * point + t;
                // And re-set in the first frame
                point = first_r.transpose() * (point - t);

                angle_images.push_back(AngleImage(std::atan2(point[0], point[1]), image_name));
              }
            }
            if (angle_images.empty())
              continue;

            // Sort the camera poses according to their angle
            std::sort(angle_images.begin(), angle_images.end());
            // Only pick views if they are far enough from each other, every 30 degrees (pi/6)
            std::vector<AngleImage>::const_iterator angle_images_iterator = angle_images.begin(), angle_images_end =
                angle_images.end();
            std::vector<AngleImage>::const_iterator previous_angle_images_iterator = angle_images_iterator;
            float current_angle = angle_images_iterator->first;
            ++angle_images_iterator;
            for (; angle_images_iterator != angle_images_end; ++angle_images_iterator, ++previous_angle_images_iterator) {
              if ((angle_images_iterator->first - current_angle) > CV_PI / 12)
              {
                current_angle = previous_angle_images_iterator->first;
                imageNames.push_back(angle_images_iterator->second);
              }
            }
          }
          std::cout << "Loading " << imageNames.size() << " images" << std::endl;

    foreach(const string& imageName, imageNames)
    {
      string f3dName = filename(name + "/" + imageName + POSTFIX);

      FileStorage fs = FileStorage(f3dName, FileStorage::READ);
      if (fs.isOpened())
      {
        Features3d f3d;
        f3d.read(fs["features3d"]);
        //TODO remove this for memory reasons, or come up with an intelligent way to
        // load up the image.
        //f3d.features().image = imread(filename(name + "/" + imageName ));
        object->observations.push_back(f3d);
      }
      fs.release();
    }
    calculateStddev(object);//, filename(name));
    objects.push_back(object);
  }
}
}
