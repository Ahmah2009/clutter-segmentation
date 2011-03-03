#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include "textured_object_detection/training_set.h"
#include "textured_object_detection/shared_functions.h"
#include "posest/pnp_ransac.h"

using namespace std;
using namespace cv;

//#define DRAWING 1

struct ChessboardCoordinates
{
  Point3f center;
  Vec3f x, y, z;

  ChessboardCoordinates(): center(0, 0, 0), x(1, 0, 0), y(0, 1, 0), z(0, 0, 1)
  {
  }
};

void normalize(Vec3f& vec)
{
  double l2norm = norm(vec);
  for (int i = 0; i < 3; i++)
    vec[i] /= l2norm;
}

void calculate3dChessCoords(const string& chessPath, ChessboardCoordinates& chessCoords)
{
  ifstream chessFile;
  chessFile.open(chessPath.c_str());
  if (!chessFile.is_open())
  {
    cout << "ERROR! Can't find file with chessboard parameters!" << endl;
    exit(-1);
  }
  vector<Point3f> chessPoints;
  while (!chessFile.eof())
  {
    Point3f chessPoint;
    chessFile >> chessPoint.x >> chessPoint.y >> chessPoint.z;
    chessPoints.push_back(chessPoint);
  }
  chessFile.close();

  chessCoords.center = chessPoints[0];
  Point3f farX = chessPoints[16];
  Point3f farY = chessPoints[3];

  chessCoords.x = farX - chessCoords.center;
  chessCoords.y = farY - chessCoords.center;
  chessCoords.z = chessCoords.x.cross(chessCoords.y);

  normalize(chessCoords.x);
  normalize(chessCoords.y);
  normalize(chessCoords.z);
}

void convertPointsToChessboardCoordinates(const vector<Point3f>& points, const ChessboardCoordinates& chessCoords, vector<Point3f>& resultedPoints)
{
  resultedPoints.clear();
  resultedPoints.resize(points.size());

  Point3f px(chessCoords.x[0], chessCoords.x[1], chessCoords.x[2]),
          py(chessCoords.y[0], chessCoords.y[1], chessCoords.y[2]),
          pz(chessCoords.z[0], chessCoords.z[1], chessCoords.z[2]);

  for (size_t pIndex = 0; pIndex < points.size(); pIndex++)
  {
      Point3f p = points[pIndex];
      p = p - chessCoords.center;
      resultedPoints[pIndex].x = p.dot(px);
      resultedPoints[pIndex].y = p.dot(py);
      resultedPoints[pIndex].z = p.dot(pz);
  }
}

Point3f calculateCenterOfMass(const vector<Point3f>& cloud)
{
  Point3f center(0.f, 0.f, 0.f);
  size_t pointsCount = cloud.size();
  for (size_t pointInd = 0; pointInd < pointsCount; pointInd++)
  {
    center += cloud[pointInd];
  }
  center = center * (1.f / pointsCount);
  return center;
}

double calculateStandardDeviation(const vector<Point3f>& cloud, const Point3f& center)
{
  double stddev;
  size_t pointsCount = cloud.size();
  if (pointsCount < 2)
  {
    stddev = 0.f;
  }
  else
  {
    double variation = 0.f;

    for (size_t pointInd = 0; pointInd < pointsCount; pointInd++)
    {
      double distance = norm(cloud[pointInd] - center);
      variation += distance * distance;
    }

    stddev = sqrt(variation / (pointsCount - 1));
  }
  return stddev;
}

void analyseImage(const string& testFolder, const vector<Point3f>& cloud,
                  const Mat& camMatrix, const Mat& distCoeffs,
                  ObjectInfo& object, int index,
                  FileStorage& fs, int& inliersCount,
                  int& falsePositivesCount, int& indexInFile)
{
  if (index < 0)
    return;

  char filename[256];
  string maskPath = testFolder + "/mask%d.png";
  sprintf(filename, maskPath.c_str(), index);
  cout << filename << endl;

  Mat mask = imread(filename, 0);
  transformMaskUsingBiggestContour(mask, false);
  int maskArea = countNonZero(mask);

#ifdef DRAWING
  Mat drawMask;
#endif

  string chessPath = testFolder + "/chess%d.txt";
  sprintf(filename, chessPath.c_str(), index);
  ChessboardCoordinates chessCoords;
  calculate3dChessCoords(filename, chessCoords);

  int oldInliersCountValue = inliersCount;

  #ifdef DRAWING
  cvtColor(mask, drawMask, CV_GRAY2BGR);
#endif

  vector<Point2f> projected;
  vector<Point3f> rotated;

  project3dPoints(cloud, object.rvec, object.tvec, rotated);

  projectPoints(Mat(rotated), Mat(3, 1, CV_64FC1, Scalar(0.0)), Mat(3, 1, CV_64FC1, Scalar(0.0)), camMatrix, distCoeffs, projected);
  Mat binMask(mask.rows, mask.cols, CV_8UC1, Scalar(0));
  vector<Point2f>::iterator it;
  for (it = projected.begin(); it != projected.end();)
  {
    if ((it->x > 0) && (it->y > 0) && (it->x < mask.cols) && (it->y < mask.rows))
    {
      circle(binMask, *it, 5, Scalar(255));
#ifdef DRAWING
      circle(drawMask, *it, 5, Scalar(255, 0, 0));
#endif
      it++;
    }
    else
      it = projected.erase(it);
  }

  dilate(binMask, binMask, Mat(), Point(-1, -1), 4);
  transformMaskUsingBiggestContour(binMask, true);


  Mat intersect;
  bitwise_and(binMask, mask, intersect);

  Mat unionImage;
  bitwise_or(binMask, mask, unionImage);

  int intersectArea = countNonZero(intersect);
  int unionArea = countNonZero(unionImage);
  int objectArea = countNonZero(binMask);

  cout << "Mask area = " << maskArea << ", object area = " << objectArea << ", intersect area = " << intersectArea << ", union area = " << unionArea << endl;
  if (intersectArea > 0.3*unionArea)
  {
    cout << "Inlier!!!" << endl;
    cout << "Object id = " << indexInFile << endl;
    vector<Point3f> convertedPoints;
    convertPointsToChessboardCoordinates(rotated, chessCoords, convertedPoints);

    Point3f centerMass = calculateCenterOfMass(convertedPoints);
    double distanceToCam = norm(centerMass) + norm(chessCoords.center);
    double stddev = calculateStandardDeviation(convertedPoints, centerMass);

    char nodeName[256];
    sprintf(nodeName, "object%d", indexInFile);
    fs << string(nodeName) << "{" << "cloud" << Mat(convertedPoints) << "index" << index << "center" << chessCoords.center <<
       "x" << chessCoords.x << "y" << chessCoords.y << "z" << chessCoords.z << "maskArea" << maskArea << "objectArea" << objectArea <<
       "intersectArea" << intersectArea << "unionArea" << unionArea << "distanceToCam" << distanceToCam << "size" << stddev << "}";
    inliersCount++;
  }
  else
  {
    cout << "FP!!!" << endl;
    falsePositivesCount++;
  }
#ifdef DRAWING
    namedWindow("mask");
    Mat smallMask;
    resize(drawMask, smallMask, Size(), 0.3, 0.3);
    imshow("mask", smallMask);

    namedWindow("intersect");
    Mat smallMaskInter;
    resize(intersect, smallMaskInter, Size(), 0.3, 0.3);
    imshow("intersect", smallMaskInter);

    Mat smallMaskUnion;
    namedWindow("union");
    resize(unionImage, smallMaskUnion, Size(), 0.3, 0.3);
    imshow("union", smallMaskUnion);

    waitKey(0);
#endif
}

bool isFileExists(const char* strFilename) {
  struct stat stFileInfo;
  bool blnReturn;
  int intStat;
  intStat = stat(strFilename,&stFileInfo);
  if(intStat == 0)
  {
    blnReturn = true;
  }
  else
  {
    blnReturn = false;
  }
  return(blnReturn);
}

int getImagesCount(const string& testFolder)
{
  char filename[256];
  int imageIndex = 1;
  while (true)
  {
    string imPath = testFolder + "/%d.png";
    sprintf(filename, imPath.c_str(), imageIndex);
    if (!isFileExists(filename))
      break;
    imageIndex++;
  }
  return imageIndex-1;
}

void fillObjectNamesVector(const string& testFolder, vector<string>& objectNames, string filename)
{
  objectNames.clear();

  ifstream nameFile;
  nameFile.open((testFolder + "/" + filename).c_str());

  if (nameFile.is_open())
  {
    while(!nameFile.eof())
    {
      string name;
      nameFile >> name;
      objectNames.push_back(name);
    }
  }

  nameFile.close();
}


class Matches
{
public:
  vector<int> imgIndexes;
  vector<int> trainIndexes;

  Matches(const string& folder)
  {
    ifstream nameFile;
    nameFile.open((folder + "/matches.txt").c_str());

    if (nameFile.is_open())
    {
      while(!nameFile.eof())
      {
        int trInd, imgInd;
        nameFile >> imgInd >> trInd;
        imgIndexes.push_back(imgInd);
        trainIndexes.push_back(trInd);
      }
    }
    nameFile.close();
  }

  int getCorrespondingTrainingIndex(const int& imgInd)
  {
    int resultIndex = -1;
    for (size_t i = 0; i < imgIndexes.size(); i++)
    {
      if (imgIndexes[i] == imgInd)
      {
        resultIndex = trainIndexes[i];
        break;
      }
    }
    return resultIndex;
  }
};

void analyseImages(const TrainingSet& tr,
                   const string& testPath,
                   const string& folderName,
                   vector<ObjectInfo>& objects, vector<int>& indexes,
                   FileStorage& resFs, int& fpCount)
{
  if (objects.size() == 0)
  {
    cout << "Objects vector is empty!" << endl;
    return;
  }

  vector<string> containedObjectNames;
  string path = testPath + "/" + folderName;
  fillObjectNamesVector(path, containedObjectNames, "objnames.txt");
  if (containedObjectNames.size() == 0)
  {
      cout << "Can't finds any object name for " << path << endl;
      return;
  }

  int fpOthers = 0;
  for (vector<ObjectInfo>::iterator objIt = objects.begin(); objIt != objects.end(); objIt++)
  {
    size_t nameIdx;
    for (nameIdx = 0; nameIdx < containedObjectNames.size(); nameIdx++)
    {
      if ((*objIt).objectName == containedObjectNames[nameIdx])
      {
        break;
      }
    }

    if (nameIdx >= containedObjectNames.size())
    {
      fpOthers++;
    }
  }
  fpCount = fpOthers;

  struct dirent* dent;
  DIR* srcdir = opendir(path.c_str());
  if (srcdir == NULL)
  {
    return;
  }

  int testIndex = 1;
  while ((dent = readdir(srcdir)) != NULL)
  {
    struct stat st;
    if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0)
      continue;
    if (fstatat(dirfd(srcdir), dent->d_name, &st, 0) < 0)
    {
      continue;
    }
    if (S_ISDIR(st.st_mode))
    {

      string objectName = string(dent->d_name);
      Matches matches(path + "/" + objectName);

      stringstream out;
      out << testIndex;
      resFs << "test" + out.str() << "{";
      resFs << "testFolder" << path + "/" + objectName;
      testIndex++;

      int imagesCount = (int)matches.imgIndexes.size(), inliersCount = 0,
          sameInliersCount = 0, falsePositivesCount = 0;

      bool isRecognize = false;
      int previousTrainIndex = -1;
      int indexInFile = 1;
      for (size_t index = 0; index < objects.size(); index++)
      {
        if (objects[index].objectName == objectName)
        {
          int trainIndex = matches.getCorrespondingTrainingIndex(indexes[index]);
          if (trainIndex != previousTrainIndex)
            isRecognize = false;

          int latestInliersCount = inliersCount;

          vector<Point3f> cloud;

          int observIndex = objects[index].imgIdx;
          for (size_t objInd = 0; objInd < tr.objNum; objInd++)
           {
             if (tr.objects[objInd]->name == objectName)
             {
               for (size_t ind = 0; ind < tr.objects[objInd]->clouds[observIndex].points.size(); ind++)
                 cloud.push_back(Point3f(tr.objects[objInd]->clouds[observIndex].points[ind].x,
                                                      tr.objects[objInd]->clouds[observIndex].points[ind].y,
                                                      tr.objects[objInd]->clouds[observIndex].points[ind].z));
               break;
             }
           }

          analyseImage(path + "/" + objectName, cloud, tr.getCameraMatrix(), tr.getDistCoeffs(),
                       objects[index], trainIndex, resFs,
                       inliersCount, falsePositivesCount, indexInFile);
          if (trainIndex == previousTrainIndex && isRecognize && inliersCount > latestInliersCount)
          {
            sameInliersCount++;
          }


          if (inliersCount > latestInliersCount)
            isRecognize = true;

          if (isRecognize)
            indexInFile++;
          previousTrainIndex = trainIndex;
        }
      }

      resFs << "imagesCount" << imagesCount;
      resFs << "allInliersCount" << inliersCount;
      resFs << "diffInliersCount" << inliersCount - sameInliersCount;
      resFs << "fpCount" << falsePositivesCount << "}";
    }
  }
  closedir(srcdir);
}

string extractBagFileName(const string& testFolder)
{
  size_t posEnd = testFolder.substr(0, testFolder.length() - 1).find_last_of('/');
  size_t posBegin = testFolder.substr(0, posEnd).find_last_of('/') + 1;
  return testFolder.substr(posBegin, posEnd - posBegin);
}


int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    printf("This is a log analyzer.\n"
      "Usage: stat\n"
      "     [-result <result.xml>]     # file with results\n"
      "     <log.txt>                  # log file with results of algorithm work\n"
      "     <testing base path>             # path to testing base\n"
      "\n");
    return 1;
  }

  string logPath = "", resPath = "result.xml", testPath = "";

  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-result") == 0)
    {
      resPath = string(argv[++i]);
    }
    else
    {
      logPath = string(argv[i++]);
      testPath = string(argv[i]);
    }
  }

  FileStorage fs;
  fs.open(logPath, FileStorage::READ);
  if (!fs.isOpened())
  {
    cout << "Can't open log file." << endl;
  }
  else
  {
    int testIndex = 1;
    string trainFolder = "";
    fs["trainFolder"] >> trainFolder;
    TrainingSet tr(trainFolder, "");
    tr.initTestCamModel(trainFolder + "/info.txt");

    FileStorage resFs;
    resFs.open(resPath, FileStorage::WRITE);

    while(true)
    {
      FileNode testNode;
      stringstream out;
      out << testIndex;
      string nodeName = "test" + out.str();
      testNode = fs[nodeName];
      if (testNode.empty())
      {
        break;
      }
      else
      {
        string testFolder = "";
        testNode["testFolder"] >> testFolder;

        string folderName = extractBagFileName(testFolder);
        cout << testFolder << "  ->   " << folderName << endl;

        FileNode objectsNode = testNode["objects"];

        vector<ObjectInfo> objects;
        vector<int> imgIndexes;
        ObjectInfo::loadObjects(objectsNode, imgIndexes, objects);

        string fileName = "file" + out.str();
        resFs << fileName << "{";
        resFs << "bagFileName" << folderName;

        int fpCount = 0;
        analyseImages(tr, testPath, folderName, objects, imgIndexes, resFs, fpCount);

        resFs << "fpCount" << fpCount;

        resFs << "}";

        testIndex++;
      }
    }

    resFs.release();
  }

  fs.release();
  return 0;
}

