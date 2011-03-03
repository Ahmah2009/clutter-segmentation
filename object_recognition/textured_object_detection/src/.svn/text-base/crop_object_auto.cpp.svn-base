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
Mat leftR, rightR, proR, leftNTImR, proIm, firstIm, firstLeftIm;
Mat drawImg, leftMask;
Mat firstMask;
bool isMaskFilled;
Mat_<Vec3f> point_cloud;
int trainIndex = 1;
string objDir, srcDir;
string train;
string objName;
string windName = "crop";
bool isForTest = false;
bool isTextured = true;
bool isIgnoreMask = false;
Mat rvec, tvec;
int currentIndex;

Mat rvecChess, tvecChess;
bool isInitChess = false;
vector<Point2f> chessPoints;
bool isSavedFirstImage = false;
vector<Point3f> chess3dPoints;


ros::Publisher cloud_pub;

vector<Point3f> frame1AllPoints;
vector<int> frame1Indices;

CameraInfo leftInfo, rightInfo, proInfo;

vector<Point> points;
Point prev_pt = Point(-1, -1);
bool is_complete = false;
double eps = 20;

vector<Point3f> firstChessObjectPoints;

void initTrainingSet(TrainingSet* tr, string ciPath)
{
  assert(tr != NULL);
  tr->isDrawInliers = false;
  tr->isDrawCorrespondence = false;
  tr->isPrintResults = false;
  tr->isDrawProjection = false;
  tr->isDrawClusters = false;
  tr->isNode = false;
  tr->initTestCamModel(ciPath);
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

Mat calcTranslation(const vector<Point3f>& points1, const vector<Point3f>& points2)
{
  assert(points1.size() == points2.size());
  Mat t = Mat::zeros(3, 1, CV_64F);
  for (size_t i = 0; i < points1.size(); i++)
  {
    t.at<double> (0, 0) += points2[i].x - points1[i].x;
    t.at<double> (1, 0) += points2[i].y - points1[i].y;
    t.at<double> (2, 0) += points2[i].z - points1[i].z;
  }

  t /= points1.size();
  return t;
}

void onMouse(int event, int x, int y, int flags, void* param)
{
  if (x >= leftNTImR.cols || y >= leftNTImR.rows)
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
      Mat drawImg1 = drawImg(Rect(0, 0, leftNTImR.cols, leftNTImR.rows));
      cvtColor(leftNTImR, drawImg1, CV_GRAY2RGB);
      leftMask.create(leftNTImR.rows, leftNTImR.cols, CV_8UC1);
      leftMask.setTo(Scalar::all(0));
      prev_pt = Point(x, y);
    }
    circle(drawImg, Point(x, y), 2, Scalar(255, 0, 0), 2, 8, 0);
    imshow(windName, drawImg);
  }
}

void savePoints()
{
  if (trainIndex == 1 && !firstMask.empty() && !isMaskFilled)
  {
    isMaskFilled = true;
    firstMask.copyTo(leftMask);
  }

  if (isMaskFilled || trainIndex != 1)
  {
    Mat im;
    proIm.copyTo(im);
    cout << "proIm size " << proIm.rows << " " << proIm.cols << endl;

    char filename[256];
    string config_path = objDir + "/%d.png";
    sprintf(filename, config_path.c_str(), trainIndex);
    imwrite(filename, im);

    vector<Point3f> object_points, rotated_object_points;

    if (trainIndex == 1)
    {
      for (int u = 0; u < point_cloud.rows; ++u)
      {
        for (int v = 0; v < point_cloud.cols; ++v)
        {
          if (isValidPoint(point_cloud.at<Vec3f> (u, v)) && leftMask.at<unsigned char> (u, v) == 255)
          {
            object_points.push_back(Point3f(point_cloud.at<Vec3f> (u, v)[0], point_cloud.at<Vec3f> (u, v)[1],
                                            point_cloud.at<Vec3f> (u, v)[2]));
          }
        }
      }
      project3dPoints(object_points, rvec, tvec, rotated_object_points);
    }
    else
    {
      for (int u = 0; u < point_cloud.rows; ++u)
      {
        for (int v = 0; v < point_cloud.cols; ++v)
        {
          if (isValidPoint(point_cloud.at<Vec3f> (u, v)))
          {
            object_points.push_back(Point3d(point_cloud.at<Vec3f> (u, v)[0], point_cloud.at<Vec3f> (u, v)[1],
                                            point_cloud.at<Vec3f> (u, v)[2]));
          }
        }
      }
      vector<Point3f> prosilica_points;
      project3dPoints(object_points, rvec, tvec, prosilica_points);
      project3dPoints(prosilica_points, rvecChess, tvecChess, rotated_object_points);

      vector<Point2d> image_points_first_cadr;
      image_points_first_cadr.resize(rotated_object_points.size());
      for (size_t i = 0; i < rotated_object_points.size(); i++)
      {
        proInfo.projectPoint(rotated_object_points[i], image_points_first_cadr[i]);
      }
      vector<int> indexes;
      for (size_t i = 0; i < image_points_first_cadr.size(); i++)
      {
        Point2d p = image_points_first_cadr[i];
        bool isPointInMask = (p.x > 0) && (p.y > 0) && (p.x < proIm.cols) && (p.y < proIm.rows);
        isPointInMask = isPointInMask && (firstMask.at<unsigned char> (p.y, p.x) == 255);
        if (isPointInMask)
        {
          indexes.push_back(1);
        }
        else
          indexes.push_back(0);
      }
      vector<Point3f> temp;
      for (size_t i = 0; i < object_points.size(); i++)
      {
        if (indexes[i])
          temp.push_back(object_points[i]);
      }
      project3dPoints(temp, rvec, tvec, rotated_object_points);
    }

    Mat binMask;
    binMask.create(im.rows, im.cols, CV_8UC1);
    binMask.setTo(Scalar::all(0));
    for (size_t i = 0; i < rotated_object_points.size(); ++i)
    {
      cv::Point2f uv;
      proInfo.projectPoint(rotated_object_points[i], uv);
      int x = (int)uv.x;
      int y = (int)uv.y;
      if (x > 0 && x < im.cols && y > 0 && y < im.rows)
      {
        for (int xx = x - 10; xx < x + 10 && xx > 0 && xx < im.cols; xx++)
          for (int yy = y - 10; yy < y + 10 && yy > 0 && yy < im.rows; yy++)
          {
            binMask.at<uint8_t> (yy, xx) = 255;
          }
      }
    }

    if (trainIndex == 1)
    {
      binMask.copyTo(firstMask);
    }

    Mat drawMask;
    bitwise_and(im, binMask, drawMask);
    string mask_path = objDir + "/mask%d.png";
    sprintf(filename, mask_path.c_str(), trainIndex);
    imwrite(filename, drawMask);

    config_path = objDir + "/%d.txt";
    sprintf(filename, config_path.c_str(), trainIndex);
    ofstream file;
    file.open(filename);
    for (size_t index = 0; index < rotated_object_points.size(); index++)
    {
      file << rotated_object_points[index].x << " " << rotated_object_points[index].y << " "
          << rotated_object_points[index].z << endl;
    }
    file.close();

    string chess3dpath = objDir + "/chess%d.txt";
    sprintf(filename, chess3dpath.c_str(), trainIndex);
    ofstream chess3dfile;
    chess3dfile.open(filename);
    for (size_t index = 0; index < chess3dPoints.size(); index++)
    {
      chess3dfile << chess3dPoints[index].x << " " << chess3dPoints[index].y << " "
          << chess3dPoints[index].z << endl;
    }
    chess3dfile.close();


    if (trainIndex == 1)
    {
      isSavedFirstImage = true;
      FileStorage fs((srcDir + "//mask.xml").c_str(), FileStorage::WRITE);
      fs << "mask" << leftMask;
      fs << "index" << currentIndex;
      fs.release();
    }
    else
    {
      config_path = objDir + "/chess%d.xml";
      sprintf(filename, config_path.c_str(), trainIndex);
      FileStorage fs(filename, FileStorage::WRITE);
      fs << "rvec" << rvecChess;
      fs << "tvec" << tvecChess;
      fs.release();
    }

    ofstream config((train + "/config.txt").c_str());
    printf("Saving to %s\n", (train + "/config.txt").c_str());
    config << objName << " " << trainIndex;
    config.close();

    ofstream matches((objDir + "/matches.txt").c_str(), ios::app);
    matches << currentIndex << " " << trainIndex << endl;
    matches.close();
    trainIndex++;
  }
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
      "Usage: crop_auto\n"
      "     [-config <file.config>]    # configuration file with algorithm parameters\n"
      "     [-transform <transform.xml>] # transform from stereo to prosilica\n"
      "     [-skip-frames <number>]    # skip <number> frames\n"
      "     [-mask-only]               # label first frame mask only\n"
      "     [-test-base]               # cropping for test base\n"
      "     [-ignore-mask]             # ignoring mask.xml file\n"
      "     <src_directory>            # should contain left and right subfolders\n"
      "     <object_name>              # object name in training base\n"
      "     <train_dir>                # folder for new training base\n"
      "\n");
    return 1;
  }
  string configPath = "", newTrDir = "", transformFile = "";
  bool maskOnly = false;
  int index = 1;
  bool readDirs = false;
  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else if (strcmp(s, "-transform") == 0)
    {
      transformFile = string(argv[++i]);
    }
    else if (strcmp(s, "-skip-frames") == 0)
    {
      index = atoi(argv[++i]);
    }
    else if(strcmp(s, "-ignore-mask") == 0)
    {
      isIgnoreMask = true;
    }
    else if(strcmp(s, "-mask-only") == 0)
    {
      maskOnly = true;
    }
    else if(strcmp(s, "-test-base") == 0)
    {
      isForTest = true;
    }
    else if(readDirs == false)
    {
      srcDir = string(argv[i++]);
      objName = string(argv[i++]);
      newTrDir = string(argv[i]);
      readDirs = true;
    }
    else
    {
      cout << "Unsupported argument: " << s << endl;
      return 1;
    }
  }

  if (!isIgnoreMask)
  {
    FileStorage mask_fs;
    mask_fs.open((srcDir + "//mask.xml").c_str(), FileStorage::READ);
    if (mask_fs.isOpened())
    {
      mask_fs["mask"] >> firstMask;
      mask_fs["index"] >> index;
    }
    mask_fs.release();
  }


  ros::init(argc, argv, "tod_train");
  ros::NodeHandle nh("~");
  cloud_pub = nh.advertise<visualization_msgs::Marker> ("clouds", 0);
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker> ("points", 0);
  TrainingSet* tr = NULL;
  bool isTrNeedUpdate = false;
  bool isRecognized = false;

  train = newTrDir;

  string lciPath, rciPath, prociPath;
  lciPath = string(srcDir) + "/left_info.txt";
  leftInfo.init(lciPath);
  rciPath = string(srcDir) + "/right_info.txt";
  rightInfo.init(rciPath);
  prociPath = string(srcDir) + "/pro_info.txt";
  proInfo.init(prociPath);

  FileStorage fs;
  string path = transformFile;
  fs.open(path.c_str(), FileStorage::READ);
  if (fs.isOpened())
  {
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;
    cout << "Read settings" << endl;
    fs.release();
  }
  else
  {
    cout << "Can't open transform parameters!" << endl;
    return 1;
  }

  string left_dir = "/left_tex", right_dir = "/right_tex";
  Mat leftIm, rightIm;

  char filename[256];
  string config_str;

  createDirIsNeeded(train);

  if (isForTest)
  {
    size_t pos = srcDir.substr(0, srcDir.length() - 1).find_last_of('/');
    string bagFileName(srcDir.substr(pos + 1));
    train = train + "/" + bagFileName;
    createDirIsNeeded(train);
  }

  objDir = train + "/" + objName;
  createDirIsNeeded(objDir);
  copyCI(prociPath, train + "/" + objName + "/info.txt");
  copyCI(prociPath, train + "/info.txt");
  if(firstMask.empty())
    namedWindow(windName);

  while (firstMask.empty() || !maskOnly)
  {
    currentIndex = index;
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

    config_str = srcDir + "/pro/%d.png";
    sprintf(filename, config_str.c_str(), index);

    proIm = imread(filename, 0);
    if (proIm.data == NULL)
      break;
    cout << filename << endl;

    Mat test;
    proIm.copyTo(test);

    bool proFound = false, leftFound = false;
    vector<Point2f> proPointBuf, leftPointBuf;
    Size boardSize(4, 5);

    proFound = findChessboardCorners(test, boardSize, proPointBuf, CV_CALIB_CB_ADAPTIVE_THRESH );
    if (proPointBuf.size() < boardSize.width * boardSize.height)
    {
      printf("Found %d points, should be %d, skipping...\n", proPointBuf.size(), boardSize.width * boardSize.height);
      proFound = 0;
    }
    if (proFound)
      cout << "Find chessboard on prosilica image" << endl;
    if (!proFound && !isInitChess)
    {
      index++;
      continue;
    }

    config_str = srcDir + "/left/%d.png";
    sprintf(filename, config_str.c_str(), index);
    Mat leftNTIm = imread(filename, 0);

    leftFound = findChessboardCorners(leftNTIm, boardSize, leftPointBuf, CV_CALIB_CB_ADAPTIVE_THRESH );
    if (leftPointBuf.size() < boardSize.width * boardSize.height)
    {
      printf("Found %d points, should be %d, skipping...\n", leftPointBuf.size(), boardSize.width * boardSize.height);
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

    if (!isForTest)
      isRecognized = recognizeImage(tr, pt_pub, test);

    if (!isRecognized || isForTest)
    {
      leftIm.copyTo(leftR);
      leftInfo.rectify(leftIm, leftR);

      rightIm.copyTo(rightR);
      rightInfo.rectify(rightIm, rightR);

      proIm.copyTo(proR);
      proInfo.rectify(proIm, proR);

      leftNTIm.copyTo(leftNTImR);
      leftInfo.rectify(leftNTIm, leftNTImR);

      Mat d, tmp;
      StereoBM(StereoBM::BASIC_PRESET, 128, 15)(leftR, rightR, d);
      d.convertTo(tmp, CV_8U, 1.0 / 16.0);

      Mat_<double> Q(4, 4, 0.0);
      initQ(Q, rightInfo);
      reprojectImageTo3D(d, point_cloud, Q, true);

      vector<Point3f> stereoPoints, objectPoints;
      vector<Point2f> imagePoints;
      cout << "chess size = " << chessPoints.size() << endl;
      for (size_t indexBuf = 0; indexBuf < leftPointBuf.size(); indexBuf++)
      {
        if (isValidPoint(point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)))
        {
          stereoPoints.push_back(Point3f(point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[0],
                                         point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[1],
                                         point_cloud.at<Vec3f> (leftPointBuf[indexBuf].y, leftPointBuf[indexBuf].x)[2]));
          imagePoints.push_back(chessPoints[indexBuf]);
        }
      }

      if (stereoPoints.size() < boardSize.width * boardSize.height)
      {
        cout << "Not enough chessboard points: " << stereoPoints.size() << endl;
        index++;
        continue;
      }

      project3dPoints(stereoPoints, rvec, tvec, objectPoints);

      chess3dPoints.clear();
      copy(objectPoints.begin(), objectPoints.end(), back_inserter(chess3dPoints));

      if (isSavedFirstImage)
      {
        solvePnP(Mat(objectPoints), Mat(imagePoints), proInfo.K, proInfo.D, rvecChess, tvecChess);

        vector<Point3f> rotated_object_points;
        project3dPoints(objectPoints, rvecChess, tvecChess, rotated_object_points);

        if (firstChessObjectPoints.size() != rotated_object_points.size())
        {
          printf("Skipping, %d chessboard points detected as opposed to %d points on the first frame\n",
                 rotated_object_points.size(), firstChessObjectPoints.size());
          index++;
          continue;
        }
        Mat tvecChess1 = calcTranslation(rotated_object_points, firstChessObjectPoints);
        printf("Adjusted translation to %f %f %f\n", tvecChess1.at<double> (0, 0), tvecChess1.at<double> (1, 0),
               tvecChess1.at<double> (2, 0));
        tvecChess += tvecChess1;

        //temporary drawing
        vector<Point3f> tempObjectPoints;
        vector<Point3f> tempRotatedObjectPoints;
        for (int u = 0; u < point_cloud.rows; ++u)
        {
          for (int v = 0; v < point_cloud.cols; ++v)
          {
            if (isValidPoint(point_cloud.at<Vec3f> (u, v)))
            {
              tempObjectPoints.push_back(Point3f(point_cloud.at<Vec3f> (u, v)[0], point_cloud.at<Vec3f> (u, v)[1],
                                                 point_cloud.at<Vec3f> (u, v)[2]));
            }
          }
        }
        project3dPoints(tempObjectPoints, rvec, tvec, tempRotatedObjectPoints);
        vector<Point2f> proImagePoints, proImagePointsOriginal;
        Mat cameraMatrix, distCoeffs;
        proInfo.getDistortionMatrix(distCoeffs);
        proInfo.getIntrinsicMatrix(cameraMatrix);
        projectPoints(Mat(tempRotatedObjectPoints), rvecChess, tvecChess, cameraMatrix, distCoeffs, proImagePoints);

        Mat rvec11, tvec11;
        rvec11.create(3, 1, CV_64FC1);
        tvec11.create(3, 1, CV_64FC1);
        rvec11.at<double> (0, 0) = rvec11.at<double> (1, 0) = rvec11.at<double> (2, 0) = 0.0;
        tvec11.at<double> (0, 0) = tvec11.at<double> (1, 0) = tvec11.at<double> (2, 0) = 0.0;
        project3dPoints(tempRotatedObjectPoints, rvecChess, tvecChess, tempObjectPoints);
        cout << tempObjectPoints[0].z << endl;
        cout << tempObjectPoints[1].z << endl;
        cout << tempObjectPoints[2].z << endl;
        projectPoints(Mat(tempObjectPoints), rvec11, tvec11, proInfo.K, proInfo.D, proImagePointsOriginal);
        Mat projImg;
        Mat projImg2;
        cvtColor(firstIm, projImg, CV_GRAY2BGR);
        cvtColor(firstIm, projImg2, CV_GRAY2BGR);

        for (size_t indexBuf = 0; indexBuf < imagePoints.size(); indexBuf++)
          circle(projImg, Point((int)imagePoints[indexBuf].x, (int)imagePoints[indexBuf].y), 40, Scalar(0, 255, 0), 5);

        for (size_t mm = 0; mm < proImagePoints.size(); mm++)
        {
          circle(projImg, Point((int)proImagePoints[mm].x, (int)proImagePoints[mm].y), 1, Scalar(255, 0, 0));
        }

        for (size_t mm = 0; mm < proImagePointsOriginal.size(); mm++)
        {
          circle(projImg2, Point((int)proImagePointsOriginal[mm].x, (int)proImagePointsOriginal[mm].y), 1, Scalar(255,
                                                                                                                  0, 0));
        }

        cout << rvecChess.at<double> (0, 0) << " " << rvecChess.at<double> (1, 0) << " " << rvecChess.at<double> (2, 0)
            << endl;
        cout << tvecChess.at<double> (0, 0) << " " << tvecChess.at<double> (1, 0) << " " << tvecChess.at<double> (2, 0)
            << endl;

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
        leftNTImR.copyTo(firstLeftIm);
      }

      if (!isSavedFirstImage && firstMask.empty())
      {
        Size size(leftR.cols + tmp.cols, MAX(leftR.rows, tmp.rows));
        drawImg.create(size, CV_MAKETYPE(leftR.depth(), 3));
        Mat drawImg1 = drawImg(Rect(0, 0, leftNTImR.cols, leftNTImR.rows));
        cvtColor(leftNTImR, drawImg1, CV_GRAY2RGB);
        Mat drawImg2 = drawImg(Rect(leftNTImR.cols, 0, tmp.cols, tmp.rows));
        cvtColor(tmp, drawImg2, CV_GRAY2RGB);

        Mat smallDrawImg;
        resize(drawImg, smallDrawImg, Size(), 1.0, 1.0);
        imshow(windName, smallDrawImg);
        cvSetMouseCallback(windName.c_str(), onMouse, 0);
      }

      if (isSavedFirstImage || !firstMask.empty())
      {
        savePoints();
        isTrNeedUpdate = true;
        if(maskOnly) break;
      }
      else
      {
        for (;;)
        {
          int c = cvWaitKey(0);
          if ((char)c == 27)
          {
            break;
          }

          if (char(c) == 32)
          {
            if (!isSavedFirstImage)
              isInitChess = false;
            points.clear();
            prev_pt = Point(-1, -1);
            break;
          }

          if ((char)c == 115)
          {
            savePoints();
            if(maskOnly) break;
            isTrNeedUpdate = true;
            break;
          }
        }
      }
    }
    if (isTrNeedUpdate)
    {
      if (tr != NULL)
        delete tr;

      if (!isForTest)
      {
        tr = new TrainingSet(train, configPath);
        initTrainingSet(tr, prociPath);
      }

      isTrNeedUpdate = false;
    }
    index++;
  }
  return 0;
}
