/*
 * ObjectConverter.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#include <fstream>
#include <sys/stat.h>
#include <dirent.h>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tod/converter/CameraConverter.h>
#include <tod/converter/ObjectConverter.h>

using namespace tod;
using namespace std;
using namespace cv;
//TODO: do threshold, when copy mask
//TODO: create pcd files

ObjectConverter::~ObjectConverter()
{
}

ObjectConverter::ObjectConverter(const string& objectName, int imageCount,
                                 const string& basePath, const string& resultPath) :
  name(objectName), count(imageCount), path(basePath), resPath(resultPath)
{
}

void invert(Mat& rvec, Mat& tvec)
{
  Mat R, RInv;
  Rodrigues(rvec, R);
  RInv = R.inv();
  Rodrigues(RInv, rvec);
  tvec = RInv*tvec;
  tvec = tvec*(-1);
}

void ObjectConverter::fillFeatures2d(Features2d& f2d, const string& digit, const string& filename)
{
  f2d.image_name = filename;
  f2d.mask_name = filename + ".mask.png";

  loadKeypoints(generatePath(digit + ".kpt"), f2d.keypoints);
  loadDescriptors(generatePath(digit + ".xml"), f2d.descriptors);

  CameraConverter::readCameraParameters(f2d.camera, generatePath("info.txt"));

  FileStorage chessFs;
  string chessPath = generatePath("chess" + digit + ".xml");
  chessFs.open(chessPath.c_str(), FileStorage::READ);
  if (chessFs.isOpened())
  {
    Mat rvec, tvec;
    chessFs["rvec"] >> rvec;
    chessFs["tvec"] >> tvec;
    invert(rvec, tvec);
    rvec.copyTo(f2d.camera.pose.rvec);
    tvec.copyTo(f2d.camera.pose.tvec);
    chessFs.release();
  }
  else
  {
    f2d.camera.pose.rvec.create(3, 1, CV_64FC1);
    f2d.camera.pose.rvec.setTo(Scalar::all(0));
    f2d.camera.pose.tvec.create(3, 1, CV_64FC1);
    f2d.camera.pose.tvec.setTo(Scalar::all(0));
  }
  f2d.camera.pose.estimated = 1;
}

void ObjectConverter::fillCloud(Cloud& cloud, const string& digit)
{
  tod::Cloud points;
  ifstream txtfile;

  string txtpath = generatePath(digit + ".txt");
  txtfile.open(txtpath.c_str());

  while (!txtfile.eof())
  {
    Point3f p;
    txtfile >> p.x >> p.y >> p.z;
    points.push_back(p);
  }

  vector<uint16_t> indices;
  loadPointsIndices(generatePath(digit + ".pi"), indices);
  cloud.clear();
  cloud.resize(indices.size());
  for (size_t ind = 0; ind < indices.size(); ind++)
  {
    cloud[ind] = points[indices[ind]];
  }
}

void ObjectConverter::convert()
{
  string listImages = "", listClouds = "";
  for (int i = 1; i <= count; i++)
  {
    stringstream digit;
    digit << i;
    string filename = digit.str() + ".png";
    listImages += filename + "\n";

    createDirIsNeeded(resPath + "/" + name);
    copyCI(generatePath(filename), generatePath(filename, true));
    copyCI(generatePath("mask" + filename), generatePath(filename + ".mask.png", true));
    string pointCloudFile = digit.str() + ".txt";
    listClouds += pointCloudFile + "\n";
    copyCI(generatePath(pointCloudFile), generatePath(pointCloudFile, true));

    Features2d f2d;
    fillFeatures2d(f2d, digit.str(), filename);

    Cloud cloud;
    fillCloud(cloud, digit.str());
    tod::Features3d f3d(f2d, cloud);

    FileStorage fs;
    fs.open(generatePath(digit.str() + ".png.f3d.yaml.gz", true), FileStorage::WRITE);
    fs << "features3d";
    f3d.write(fs);
    fs.release();
  }

  ofstream file;
  file.open(generatePath("images.txt", true).c_str());
  file << listImages.substr(0, listImages.length() - 1);
  file.close();

  ofstream pcdsFile;
  pcdsFile.open(generatePath("pcds.txt", true).c_str());
  pcdsFile << listClouds.substr(0, listClouds.length() - 1);
  pcdsFile.close();
}

string ObjectConverter::generatePath(const string& filename, bool useResPath)
{
  string result;
  if (useResPath)
    result = resPath;
  else
    result = path;
  result += "/" + name + "/" + filename;
  return result;
}

void ObjectConverter::loadPointsIndices(const string& filename, vector<uint16_t>& indices)
{
  indices.clear();

  size_t size;

  ifstream piFile;
  piFile.open(filename.c_str());
  piFile >> size;
  indices.resize(size);
  for (size_t ind = 0; ind < size; ind++)
    piFile >> indices[ind];
  piFile.close();
}

void ObjectConverter::loadDescriptors(const string& filename, Mat& descriptors)
{
  FileStorage fs;
  fs.open(filename.c_str(), FileStorage::READ);
  fs["descriptor"] >> descriptors;
  fs.release();
}

void ObjectConverter::loadKeypoints(const string& filename, vector<KeyPoint>& keypoints)
{
  keypoints.clear();

  size_t size;

  ifstream kptFile;
  kptFile.open(filename.c_str());
  kptFile >> size;
  keypoints.resize(size);
  for (size_t ind = 0; ind < size; ind++)
    kptFile >> keypoints[ind].pt.y >> keypoints[ind].pt.x;
  kptFile.close();
}

void ObjectConverter::createDirIsNeeded(string path)
{
  DIR* dir = opendir(path.c_str());
  if (dir == NULL)
  {
    mkdir(path.c_str(), 0777);
  }
  else
    closedir(dir);
}

void ObjectConverter::copyCI(string from, string to)
{
  ifstream ifs(from.c_str(), ios::binary);
  ofstream ofs(to.c_str(), ios::binary);

  ofs << ifs.rdbuf();
  ifs.close();
  ofs.close();
}

