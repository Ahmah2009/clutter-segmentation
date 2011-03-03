#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include "textured_object_detection/training_set.h"
#include "textured_object_detection/shared_functions.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "posest/pnp_ransac.h"

using namespace cv;
using namespace std;

char* name, *dir;
Mat leftR, rightR, testR, proIm, firstIm;
Mat drawImg, leftMask;
bool isMaskFilled;
Mat_<Vec3f> point_cloud;
int trainIndex = 1;
int testIndex = 1;
string objDir;
string train;
string objName;
string windName = "crop";
bool isTextured = true;
bool isUsedProsilica = false;
Mat rvec, tvec;
Mat rvecChess, tvecChess;
bool isInitChess = false;
vector<Point2f> chessPoints;
ros::Publisher cloud_pub;

vector<Point3f> frame1AllPoints;
vector<int> frame1Indices;

CameraInfo leftInfo, rightInfo, proInfo;

vector<Point> points;
Point prev_pt = Point(-1, -1);
bool is_complete = false;
double eps = 20;

vector<Point3f> firstChessObjectPoints;

Point3f massCenter(const vector<Point3f>& points)
{
  Point3f sum = Point3f(0, 0, 0);
  for(size_t i = 0; i < points.size(); i++) sum += points[i];
  sum *= (1.0/points.size());
  return sum;
}

Mat calcTranslation(const vector<Point3f>& points1, const vector<Point3f>& points2)
{
  assert(points1.size() == points2.size());
  Mat t = Mat::zeros(3, 1, CV_64F);
  for(size_t i = 0; i < points1.size(); i++)
  {
    t.at<double>(0, 0) += points2[i].x - points1[i].x;
    t.at<double>(1, 0) += points2[i].y - points1[i].y;
    t.at<double>(2, 0) += points2[i].z - points1[i].z;
  }

  t /= points1.size();
  return t;
}

void onMouse(int event, int x, int y, int flags, void* param)
{
  if (x >= testR.cols || y >= testR.rows)
    return;

  if (event == CV_EVENT_LBUTTONDOWN)
  {
    if (prev_pt.x >= 0)
    {
      points.push_back(prev_pt);
      Point pt = Point(x, y);
      line(drawImg, prev_pt, pt, Scalar(255, 0, 0), 2, 8, 0);
      if (norm(pt - points[0]) < eps)
      {
        points.push_back(pt);
        points.push_back(points[0]);
        line(drawImg, pt, points[0], Scalar(255, 0, 0), 2, 8, 0);
        prev_pt = Point2f(-1, -1);
        leftMask.setTo(Scalar::all(0));
        Point* ps = new Point[points.size()];
        for (size_t i = 0; i < points.size(); i++)
        {
          ps[i] = points[i];
        }
        fillConvexPoly(leftMask, ps, points.size(), Scalar(255));
        delete[] ps;
        isMaskFilled = true;
      }
      else
        prev_pt = pt;
    }
    else
    {
      points.clear();
      isMaskFilled = false;
      Mat drawImg1 = drawImg(Rect(0, 0, testR.cols, testR.rows));
      cvtColor(testR, drawImg1, CV_GRAY2RGB);
      leftMask.create(testR.rows, testR.cols, CV_8UC1);
      leftMask.setTo(Scalar::all(0));
      prev_pt = Point(x, y);
    }
    circle(drawImg, Point(x, y), 2, Scalar(255, 0, 0), 2, 8, 0);
    imshow(windName, drawImg);
  }
}

void savePoints(TrainingSet* tr)
{
  if (isMaskFilled)
  {
    Mat im;
    if (!isUsedProsilica)
    {
      testR.copyTo(im);
      bitwise_and(testR, leftMask, im);
    }
    else
    {
      proIm.copyTo(im);
      cout << "proIm size " << proIm.rows << " " << proIm.cols << endl;
    }
    char filename[256];
    string config_path = objDir + "/%d.png";
    sprintf(filename, config_path.c_str(), trainIndex);
    imwrite(filename, im);

    vector<Point3f> object_points;
    for (int u = 0; u < point_cloud.rows; ++u)
     {
       for (int v = 0; v < point_cloud.cols; ++v)
       {
         if (isValidPoint(point_cloud.at<Vec3f> (u, v)) && leftMask.at<unsigned char> (u, v) == 255)
         {
           object_points.push_back(Point3f(point_cloud.at<Vec3f> (u, v)[0],
                                   point_cloud.at<Vec3f> (u, v)[1],
                                   point_cloud.at<Vec3f> (u, v)[2]));
         }
       }
     }

    vector<Point3f> rotated_object_points;
    if (isUsedProsilica)
      project3dPoints(object_points, rvec, tvec, rotated_object_points);

    config_path = objDir + "/%d.txt";
    sprintf(filename, config_path.c_str(), trainIndex);
    ofstream file;
    file.open(filename);

    for (size_t index = 0; index < object_points.size(); index++)
    {
      if (isUsedProsilica)
        file << rotated_object_points[index].x << " " << rotated_object_points[index].y << " " << rotated_object_points[index].z << endl;
      else
        file << object_points[index].x << " " << object_points[index].y << " " << object_points[index].z << endl;
    }
    file.close();

#if 1
    Mat drawImg;
    firstIm.copyTo(drawImg);
    vector<Point2f> projectedPoints;
    Mat _rvecChess = rvecChess, _tvecChess = tvecChess;
    if(trainIndex == 1)
    {
      _rvecChess = Mat::zeros(3, 1, CV_64F);
      _tvecChess = Mat::zeros(3, 1, CV_64F);
    }
    projectPoints(Mat(rotated_object_points), _rvecChess, _tvecChess, proInfo.K, proInfo.D, projectedPoints);
    for(size_t i = 0; i < projectedPoints.size(); i++)
    {
      circle(drawImg, Point(projectedPoints[i].x, projectedPoints[i].y), 5, cvScalar(0));
    }

    vector<Point3f> frame1Points;
    project3dPoints(rotated_object_points, _rvecChess, _tvecChess, frame1Points);

    frame1AllPoints.insert(frame1AllPoints.end(), frame1Points.begin(), frame1Points.end());
    frame1Indices.insert(frame1Indices.end(), frame1Points.size(), trainIndex);

    tr->publishPoints(frame1AllPoints, cloud_pub, frame1Indices);

#if 0
    namedWindow("1", 1);
    imshow("1", drawImg);
#endif
//    waitKey(0);
#endif

#ifdef CHESSBOARD
    if (trainIndex > 1)
    {
      config_path = objDir + "/chess%d.xml";
      sprintf(filename, config_path.c_str(), trainIndex);
      FileStorage fs(filename, FileStorage::WRITE);
      fs << "rvec" << rvecChess;
      fs << "tvec" << tvecChess;
      fs.release();
    }
#endif

    ofstream config((train + "/config.txt").c_str());
    printf("Saving to %s\n", (train + "/config.txt").c_str());
    config << objName << " " << trainIndex;
    config.close();

    trainIndex++;
  }
}

void initDirectories(const string dir, string& left_dir, string& right_dir)
{
  string left = "/left", right = "/right", leftt = "/left_tex", rightt = "/right_tex";
  string tex_dir = dir + leftt;
  DIR* srcdir = opendir(tex_dir.c_str());
  if (srcdir == NULL)
  {
    isTextured = false;
    srcdir = opendir((dir + left).c_str());
    assert(srcdir != NULL);
    closedir(srcdir);
  }
  else
    closedir(srcdir);

  if (isTextured)
  {
    left_dir = leftt;
    right_dir = rightt;
  }
  else
  {
    left_dir = left;
    right_dir = right;
  }
}

void initTrainingSet(TrainingSet* tr, string leftciPath)
{
  assert(tr != NULL);
  tr->isDrawInliers = false;
  tr->isDrawCorrespondence = false;
  tr->isPrintResults = false;
  tr->isDrawProjection = false;
  tr->isDrawClusters = false;
  tr->isNode = false;
  tr->initTestCamModel(leftciPath);
}

bool recognizeImage(TrainingSet* tr, const ros::Publisher& pt_pub, const Mat& test)
{
  if (tr == NULL)
    return false;

  vector<ObjectInfo> objects;
  cout << "Recognizing..." << endl;
  tr->recognize(test, objects, pt_pub);

  if (objects.size() >= 1)
  {
    if (objects[0].objectId == 0)
      return true;
    else
      return false;
  }
  else
    return false;
}

void copyCI(string from, string to)
{
  std::ifstream ifs(from.c_str(), std::ios::binary);
  std::ofstream ofs(to.c_str(), std::ios::binary);

  ofs << ifs.rdbuf();
  ifs.close();
  ofs.close();
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    printf("This is a tool for creating training base.\n"
      "Usage: crop\n"
      "     [-config <file.config>]    # configuration file with algorithm parameters\n"
      "     [-prosilica <transform.xml>] # transform from stereo to prosilica\n"
      "     [-skip-frames <number>]    # skip <number> frames\n"
      "     <src_directory>            # should contain left and right subfolders\n"
      "     <object_name>              # object name in training base\n"
      "     <train_dir>                # folder for new training base\n"
      "\n");
    return 1;
  }

  string srcDir = "", configPath = "", newTrDir = "", transformFile = "";
  int index = 1;

  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else if (strcmp(s, "-prosilica") == 0)
    {
      isUsedProsilica = true;
      transformFile = string(argv[++i]);
    }
    else if(strcmp(s, "-skip-frames") == 0)
    {
      index = atoi(argv[++i]);
    }
    else
    {
      srcDir = string(argv[i++]);
      objName = string(argv[i++]);
      newTrDir = string(argv[i]);
    }
  }

  train = newTrDir;

  string lciPath, rciPath, prociPath;
  lciPath = string(srcDir) + "/left_info.txt";
  leftInfo.init(lciPath);
  rciPath = string(srcDir) + "/right_info.txt";
  rightInfo.init(rciPath);
  prociPath = string(srcDir) + "/pro_info.txt";
  if (isUsedProsilica)
  {
    proInfo.init(prociPath);
    FileStorage fs;
    string path = transformFile;//srcDir + "/transform.xml";
    fs.open(path.c_str(), FileStorage::READ);
    if (fs.isOpened()){
      fs["rvec"] >> rvec;
      fs["tvec"] >> tvec;
      fs.release();
    }
    else
    {
      cout << "Can't open transform parameters!" << endl;
      return 1;
    }
  }

  string left_dir, right_dir;
  initDirectories(srcDir, left_dir, right_dir);

  Mat leftIm, rightIm;

  char filename[256];
  string config_str;

  ros::init(argc, argv, "tod_train");
  ros::NodeHandle nh("~");
  cloud_pub = nh.advertise<visualization_msgs::Marker> ("clouds", 0);
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker> ("points", 0);
  TrainingSet* tr = NULL;
  bool isTrNeedUpdate = false;
//  int index = 40;
  bool isRecognized = false;

  createDirIsNeeded(train);
  objDir = train + "/" + objName;
  createDirIsNeeded(objDir);
  //createDirIsNeeded(train + "/test");
  if (isUsedProsilica)
  {
    copyCI(prociPath, train + "/" + objName + "/info.txt");
    copyCI(prociPath, train + "/info.txt");
  }
  else
  {
    copyCI(lciPath, train + "/" + objName + "/info.txt");
    copyCI(lciPath, train + "/info.txt");
  }

  namedWindow(windName);

  while (true)
  {
    config_str = srcDir + left_dir + "/%d.png";
    sprintf(filename, config_str.c_str(), index);
    leftIm = imread(filename, 0);
    if (leftIm.data == NULL)
      break;
    cout << filename << endl;


    config_str = srcDir + right_dir + "/%d.png";
    sprintf(filename, config_str.c_str(), index);

    rightIm = imread(filename, 0);
    if (rightIm.data == NULL)
      break;
    cout << filename << endl;

    if (isUsedProsilica)
    {
      config_str = srcDir + "/pro/%d.png";
      sprintf(filename, config_str.c_str(), index);

      proIm = imread(filename, 0);
      if (proIm.data == NULL)
        break;
      cout << filename << endl;
    }

    Mat test;
    if (!isUsedProsilica)
    {
      if (isTextured)
      {
        config_str = srcDir + "/left/%d.png";
        sprintf(filename, config_str.c_str(), index);
        test = imread(filename, 0);
      }
      else
        leftIm.copyTo(test);
    }
    else
    {
      proIm.copyTo(test);
    }
    isRecognized = recognizeImage(tr, pt_pub, test);

    bool proFound = false, leftFound = false;
    vector<Point2f> proPointBuf, leftPointBuf;
    Size boardSize(4, 5);

#ifdef CHESSBOARD
    if (isUsedProsilica)
    {
      proFound = findChessboardCorners(test, boardSize, proPointBuf, CV_CALIB_CB_ADAPTIVE_THRESH );

      if(proPointBuf.size() < boardSize.width*boardSize.height)
      {
        printf("Found %d points, should be %d, skipping...\n", proPointBuf.size(), boardSize.width*boardSize.height);
        proFound = 0;
      }
      if (proFound)
        cout << "Find chessboard on prosilica image" << endl;
      if (!proFound && tr == NULL)
      {
        index++;
        continue;
      }
    }
#endif

    if (isUsedProsilica)
    {
      if (isTextured)
      {
        config_str = srcDir + "/left/%d.png";
        sprintf(filename, config_str.c_str(), index);
        test = imread(filename, 0);
      }
      else
        leftIm.copyTo(test);
    }

#ifdef CHESSBOARD
    if (isUsedProsilica)
    {
      leftFound = findChessboardCorners(test, boardSize, leftPointBuf, CV_CALIB_CB_ADAPTIVE_THRESH );
      if(leftPointBuf.size() < boardSize.width*boardSize.height)
      {
        printf("Found %d points, should be %d, skipping...\n", leftPointBuf.size(), boardSize.width*boardSize.height);
        leftFound = 0;
      }

      if (leftFound)
        cout << "Find chessboard on left image" << endl;
      if (!leftFound)
      {
        index++;
        continue;
      }
      if (leftFound && proFound)
      {
        if (isInitChess == false)
        {
          chessPoints = proPointBuf;
          proIm.copyTo(firstIm);
          isInitChess = true;
        }
      }
    }
#endif

    if (!isRecognized)
    {
      leftIm.copyTo(leftR);
      leftInfo.rectify(leftIm, leftR);

      if (isTextured)
      {
        test.copyTo(testR);
        leftInfo.rectify(test, testR);
      }
      else
      {
        leftR.copyTo(testR);
      }

      rightIm.copyTo(rightR);
      rightInfo.rectify(rightIm, rightR);

      Mat d, tmp;
      StereoBM(StereoBM::BASIC_PRESET, 128, 15)(leftR, rightR, d);
      d.convertTo(tmp, CV_8U, 1.0 / 16.0);

      Mat_<double> Q(4, 4, 0.0);
      initQ(Q, rightInfo);
      reprojectImageTo3D(d, point_cloud, Q, true);

#ifdef CHESSBOARD
      vector<Point3f> stereoPoints, objectPoints;
      vector<Point2f> imagePoints;
      if (isUsedProsilica)
      {
        for (size_t indexBuf = 0; indexBuf < leftPointBuf.size(); indexBuf++)
        {
          if (isValidPoint(point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)))
          {
            stereoPoints.push_back(
                                   Point3f(
                                           point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[0],
                                           point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[1],
                                           point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[2]));
            imagePoints.push_back(chessPoints[indexBuf]);
          }
        }


        if (stereoPoints.size() < boardSize.width*boardSize.height)
        {
          cout << "Not enough chessboard points: " << stereoPoints.size() << endl;
          index++;
          continue;
        }
        project3dPoints(stereoPoints, rvec, tvec, objectPoints);
        Point3f center = massCenter(objectPoints);
        printf("mass center: %f %f %f\n", center.x, center.y, center.z);

        if (tr != NULL)
        {
          solvePnP(Mat(objectPoints), Mat(imagePoints), proInfo.K, proInfo.D, rvecChess, tvecChess);

          vector<Point3f> rotated_object_points;
          project3dPoints(objectPoints, rvecChess, tvecChess, rotated_object_points);

          if(firstChessObjectPoints.size() != rotated_object_points.size())
          {
            printf("Skipping, %d chessboard points detected as opposed to %d points on the first frame\n",
                   rotated_object_points.size(), firstChessObjectPoints.size());
            index++;
            continue;
          }
          Mat tvecChess1 = calcTranslation(rotated_object_points, firstChessObjectPoints);
          printf("Adjusted translation to %f %f %f\n", tvecChess1.at<double>(0, 0), tvecChess1.at<double>(1, 0), tvecChess1.at<double>(2, 0));
          tvecChess += tvecChess1;

          vector<Point2f> projected_points;
          projected_points.resize(rotated_object_points.size());
          Mat rvec1, tvec1;
          rvec1.create(3, 1, CV_64FC1);
          tvec1.create(3, 1, CV_64FC1);
          rvec1.at<double> (0, 0) = rvec1.at<double> (1, 0) = rvec1.at<double> (2, 0) = 0.0;
          tvec1.at<double> (0, 0) = tvec1.at<double> (1, 0) = tvec1.at<double> (2, 0) = 0.0;
          projectPoints(Mat(rotated_object_points), rvec1, tvec1, proInfo.K, proInfo.D, projected_points);

          float error = 0;
          float avg_error = 0;
          cout << "Projected points count " << projected_points.size() << endl;
          for (size_t indexBuf = 0; indexBuf < projected_points.size(); indexBuf++)
          {
            float dist = norm(projected_points[indexBuf] - imagePoints[indexBuf]);
            if (dist > error)
            {
              error = dist;
            }

            avg_error += dist * dist;
          }
          cout << "Max error = " << error << endl;
          if (error > 10)
          {
            cout << "Too big error!" << endl;
            index++;
            continue;
          }
          cout << "Average error = " << sqrt(avg_error / projected_points.size()) << endl;
        }
        else
        {
          firstChessObjectPoints = objectPoints;
        }
      }
#endif

      Size size(leftR.cols + tmp.cols, MAX(leftR.rows, tmp.rows));
      drawImg.create(size, CV_MAKETYPE(leftR.depth(), 3));
      Mat drawImg1 = drawImg(Rect(0, 0, testR.cols, testR.rows));
      cvtColor(testR, drawImg1, CV_GRAY2RGB);
      Mat drawImg2 = drawImg(Rect(testR.cols, 0, tmp.cols, tmp.rows));
      cvtColor(tmp, drawImg2, CV_GRAY2RGB);

      Mat smallDrawImg;
      resize(drawImg, smallDrawImg, Size(), 1.0, 1.0);
      imshow(windName, smallDrawImg);
      cvSetMouseCallback(windName.c_str(), onMouse, 0);
      for (;;)
      {
        int c = cvWaitKey(0);
        if ((char)c == 27)
          break;

        if (char(c) == 32)
        {
          points.clear();
          prev_pt = Point(-1, -1);
          break;
        }

        if ((char)c == 115)
        {
          savePoints(tr);
          isTrNeedUpdate = true;
          break;
        }
      }
    }

    if (isTrNeedUpdate)
    {
      if (tr != NULL)
        delete tr;
      tr = new TrainingSet(train, configPath);
      if (!isUsedProsilica)
        initTrainingSet(tr, lciPath);
      else
        initTrainingSet(tr, prociPath);
      isTrNeedUpdate = false;
    }
    index++;
  }
  return 0;
}
