/*
 * Calculator.cpp
 *
 *  Created on: Feb 5, 2011
 *      Author: Alexander Shishkov
 *
 */

#include <tod/test/Calculator.h>
#include <tod/detecting/Loader.h>
#include <tod/detecting/Tools.h>

#include <boost/foreach.hpp>
#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations

#define foreach BOOST_FOREACH

using namespace tod;
using namespace cv;
namespace bfs = boost::filesystem;

Calculator::Calculator(const CalculatorParameters& params_)
{
  params = params_;
}

string Calculator::extractTestBasePath(const string& testFolder)
{
  size_t end = testFolder.substr(0, testFolder.length() - 1).find_last_of('/');
  return testFolder.substr(0, end);
}

void Calculator::drawProjection(Mat& out, const ExtendedObjectInfo& objectInfo,
                    const Ptr<TexturedObject>& object)
{
  Guess guess(object, PoseRT(objectInfo.object.rvec, objectInfo.object.tvec), out);
  guess.imageIndex = objectInfo.object.imgIdx;
  guess.draw(out, 0, params.trainDirectory);
}

void Calculator::generateGuessMask(const ExtendedObjectInfo& objectInfo, const TrainingBase& base, int rows, int cols, Mat& binMask)
{
  const Ptr<TexturedObject>& object = base.getObject(objectInfo.object.objectId);
  vector<Point2f> projectedPoints;
  if (objectInfo.object.imgIdx < 0)
  {
    vector<Point3f> rotatedPoints;
    vector<Point3f> cloud;
    PoseRT inverted = Tools::invert(object->observations[0].features().camera.pose);
    project3dPoints(Mat(object->observations[0].cloud()), inverted.rvec, inverted.tvec, cloud);
    project3dPoints(Mat(cloud), objectInfo.object.rvec, objectInfo.object.tvec, rotatedPoints);
    Tools::filterOutlierPoints(rotatedPoints, 0.9);
    projectPoints(Mat(rotatedPoints), Mat(3, 1, CV_64F, Scalar(0.0)), Mat(3, 1, CV_64F, Scalar(0.0)),
                  object->observations[0].camera().K, object->observations[0].camera().D, projectedPoints);
  }
  else
  {
    vector<Point3f> objectPoints;
    objectPoints = object->observations[objectInfo.object.imgIdx].cloud();
    Tools::filterOutlierPoints(objectPoints, 0.9);
    projectPoints(Mat(objectPoints), objectInfo.object.rvec,
                 objectInfo.object.tvec, object->observations[objectInfo.object.imgIdx].camera().K,
                object->observations[objectInfo.object.imgIdx].camera().D, projectedPoints);
  }

  binMask.create(rows, cols, CV_8UC1);
  binMask = Scalar(0);
  for (vector<Point2f>::iterator point = projectedPoints.begin(); point != projectedPoints.end();)
  {
    if (point->x >= cols || point->y >= rows || point->x < 0 || point->y < 0)
      point = projectedPoints.erase(point);
    else
    {
      circle(binMask, *point, 2, Scalar(255));
      point++;
    }
  }
  vector<Point2f> hull;
  if (projectedPoints.size() > 0)
  {
    convexHull(Mat(projectedPoints), hull);
    vector<Point> ihull;
    for (size_t i = 0; i < hull.size(); i++)
    {
      ihull.push_back(Point(hull[i].x, hull[i].y));
    }

    Point* points = (Point*)&ihull[0];
    int npoints = ihull.size();
    fillPoly(binMask, const_cast<const Point**>(&points), &npoints, 1, Scalar(255));
  }
}

bool Calculator::isInlier(const Mat& mask, const Mat& guessMask)
{
  Mat intersectImage;
  bitwise_and(guessMask, mask, intersectImage);

  Mat unionImage;
  bitwise_or(guessMask, mask, unionImage);

  int intersectArea = countNonZero(intersectImage);
  int unionArea = countNonZero(unionImage);

  if (params.verbose == 2)
  {
    Mat smallGuessMask;
    resize(guessMask, smallGuessMask, Size(), 0.3, 0.3);
    namedWindow("guess");
    imshow("guess", smallGuessMask);
    Mat smallMask;
    resize(mask, smallMask, Size(), 0.3, 0.3);
    namedWindow("mask");
    imshow("mask", smallMask);
    Mat smallInterImage;
    resize(intersectImage, smallInterImage, Size(), 0.3, 0.3);
    namedWindow("intersection");
    imshow("intersection", smallInterImage);
    Mat smallUnionImage;
    resize(unionImage, smallUnionImage, Size(), 0.3, 0.3);
    namedWindow("union");
    imshow("union", smallUnionImage);
    waitKey(0);
  }

  bool result = false;
  if (intersectArea > 0.4*unionArea)
  {
    result = true;
  }
  return result;
}

void Calculator::addToMap(ImageMap& map, const string& key)
{
  if (map.find(key) == map.end())
  {
    map[key] = 1;
  }
  else
  {
    map[key]++;
  }
}

void Calculator::getObjects(const string& testFolder, vector<string>& objects)
{
  objects.clear();
  if (!bfs::exists(testFolder))
    return;
  bfs::directory_iterator end_itr;
  for (bfs::directory_iterator itr(testFolder); itr != end_itr; ++itr)
  {
    if (bfs::is_directory(itr->status()))
    {
      string name = itr->path().filename();
      if (name == "dump" || name == "images")
        continue;
      objects.push_back(name);
    }
  }
}

int Calculator::getImagesCount(string directory)
{
  int count = 0;
  bfs::directory_iterator end_itr;
  for (bfs::directory_iterator itr(directory); itr != end_itr; ++itr)
  {
    size_t position = itr->leaf().rfind(".png");
    if (position!=string::npos)
    {
      count++;
    }
  }
  return count;
}

int Calculator::getCount(ImageMap& map)
{
  int count = 0;
  for(ImageMap::iterator inlier = map.begin(); inlier != map.end(); inlier++)
  {
    count += 1;
  }
  return count;
}

int Calculator::getSum(ImageMap& map)
{
  int count = 0;
  for(ImageMap::iterator inlier = map.begin(); inlier != map.end(); inlier++)
  {
    count += inlier->second;
  }
  return count;
}

void Calculator::calculateDetectionRate(FileNode& testNode, TrainingBase& base)
{
  string imageFolder = "";
  testNode["testFolder"] >> imageFolder;

  string testFolder = extractTestBasePath(imageFolder);

  FileNode objectsNode = testNode["objects"];

  vector<ExtendedObjectInfo> objects;
  ExtendedObjectInfo::loadObjects(objectsNode, objects);

  vector<ImageMap> inliers, falsePositives;
  inliers.resize(base.size());
  falsePositives.resize(base.size());

  foreach (ExtendedObjectInfo& objectInfo, objects)
  {
    string imagePath = imageFolder + "/" + objectInfo.imageFile;
    string objectDirectory = testFolder + "/" + objectInfo.object.objectName;

    Mat mask = imread(objectDirectory + "/" + objectInfo.imageFile + ".mask.png", 0);

    if (params.verbose)
    {
      Mat image = imread(imagePath, 1);
      const Ptr<TexturedObject> object = base.getObject(objectInfo.object.objectId);
      drawProjection(image, objectInfo, object);
      Mat smallImage;
      resize(image, smallImage, Size(), 0.3, 0.3);
      namedWindow("projection");
      imshow("projection", smallImage);
      waitKey(0);
    }

    if (mask.empty())
    {
      if (!boost::filesystem::exists(objectDirectory))
        addToMap(falsePositives[objectInfo.object.objectId], objectInfo.imageFile);
      continue;
    }

    Mat guessMask;
    generateGuessMask(objectInfo, base, mask.rows, mask.cols, guessMask);

    if (isInlier(mask, guessMask))
    {
      addToMap(inliers[objectInfo.object.objectId], objectInfo.imageFile);
    }
    else
    {
      addToMap(falsePositives[objectInfo.object.objectId], objectInfo.imageFile);
    }
  }

  vector<string> testObjects;
  getObjects(testFolder, testObjects);
  cout << "<name>\t<detection_rate>\t<false positive rate>\t<re-found inliers>" << endl;
  foreach(const string& name, testObjects)
  {
    vector<int> objectIds;
    base.getObjectIds(objectIds);
    foreach(int id, objectIds)
    {
      const Ptr<TexturedObject> object = base.getObject(id);
      if (object->name == name)
      {
        int imagesCount = getImagesCount(testFolder + "/" + name);
        int inliersCount = getCount(inliers[id]);
        int fpsCount = getCount(falsePositives[id]);
        int inliersSum = getSum(inliers[id]);
        cout << name << "\t" << inliersCount/(float)imagesCount << "\t" << fpsCount/(float)imagesCount <<
            "\t" << (inliersSum - inliersCount)/(float)imagesCount << endl;
      }
    }
  }
}

void Calculator::calculate()
{
  FileStorage fs;
  fs.open(params.logFilename, FileStorage::READ);
  if (!fs.isOpened())
  {
    cout << "Can't open log file." << endl;
    return;
  }

  fs["trainFolder"] >> params.trainDirectory;

  tod::Loader loader(params.trainDirectory);
  vector<cv::Ptr<TexturedObject> > objects;
  loader.readTexturedObjects(objects);

  if (!objects.size())
  {
    cout << "Empty base\n" << endl;
    return;
  }

  TrainingBase base(objects);

  for (int testIndex = 1; true; testIndex++)
  {
    FileNode testNode;
    stringstream out;
    out << testIndex;
    testNode = fs["test" + out.str()];
    if (testNode.empty())
    {
      break;
    }
    else
    {
      calculateDetectionRate(testNode, base);
    }
  }

  fs.release();
}
