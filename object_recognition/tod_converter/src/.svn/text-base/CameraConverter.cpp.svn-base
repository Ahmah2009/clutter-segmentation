/*
 * CameraConverter.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#include <fstream>
#include <opencv2/core/core.hpp>

#include <tod/converter/CameraConverter.h>

using namespace tod;
using namespace cv;
using namespace std;

CameraConverter::CameraConverter(string basePath, string resultPath): path(basePath), resPath(resultPath),
    cameraFileName("/info.txt"), resultFile("/camera.yaml"), yamlNodeName("camera")
{
}

CameraConverter::~CameraConverter()
{
}

void CameraConverter::convert()
{
  Camera camera;
  readCameraParameters(camera, path + cameraFileName);
  writeCameraParameters(camera, resPath + resultFile);
}

void CameraConverter::writeCameraParameters(const Camera& cam, const string& filename)
{
  FileStorage fs;
  fs.open(filename.c_str(), FileStorage::WRITE);
  fs << yamlNodeName;
  cam.write(fs);
  fs.release();
}

void CameraConverter::readCameraParameters(Camera& cam, const string& filename)
{
  ifstream camfile;
  camfile.open(filename.c_str());
  if (camfile.is_open())
  {
    camfile >> cam.image_size.height >> cam.image_size.width;
    cam.D.create(1, 5, CV_64FC1);
    for (int i = 0; i < 5; i++)
      camfile >> cam.D.at<double> (0, i);
    cam.K.create(3, 3, CV_64FC1);
    for (int i = 0; i < 9; i++)
      camfile >> cam.K.at<double> (i / 3, i % 3);
    camfile.close();
  }
}
