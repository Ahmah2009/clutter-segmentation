#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include "opencv/highgui.h"
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <textured_object_detection/training_set3d.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include "posest/pnp_ransac.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <textured_object_detection/shared_functions.h>

using namespace cv;
using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;

string windName = "crop";

struct MaskParams
{
  Mat drawImg;
  Mat img;
  Mat binaryMask;
  vector<Point> points;
  Point prev_pt;
  bool is_complete;
  double eps;

  MaskParams()
  {
    prev_pt = Point(-1, -1);
    is_complete = false;
    eps = 20.f;
  }
};

class CropSaver
{
public:
  MaskParams maskParams;
  CropSaver(const ros::NodeHandle &_nh, string _cloudTopic, string _imageTopic, string _ciTopic, string _configPath,
            string _newTrDir, string _objName, bool _isIgnoreRecognize);
  virtual ~CropSaver();
  void init();
private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void cloudCallback(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
  void savePoints();

  sensor_msgs::CvBridge bridge;
  ros::Publisher pt_pub;
  vector<ObjectInfo> objects;
  image_transport::Publisher pub;
  ros::Subscriber ci_sub;

  string ciTopic;
  string ciPath;
  string cloudTopic, imageTopic, configPath, newTrDir, objName;
  bool isIgnoreRecognize;
  bool isTrNeedUpdate;
  string objDir;
  vector<Point2f> chessPoints;
  vector<Point3f> firstChessObjectPoints;

  vector<Point3f> cloudPoints;
  Mat mask;

  vector<Point3f> chess3dPoints;

  Mat rvecChess, tvecChess;

  int trainIndex;

  Subscriber<sensor_msgs::Image> image_sub;
  Subscriber<sensor_msgs::PointCloud2> cloud_sub;
  message_filters::Synchronizer<SyncPolicy> sync;
  int queue_size;
  Mat firstView;

  bool isCamInit, isSavedFirstImage;

  TrainingSet* tr;
  CameraInfo cameraInfo;

  ros::NodeHandle nh;
};

void onMouse(int event, int x, int y, int flags, void* _params)
{
  MaskParams* params = (MaskParams*)_params;
  if (event == CV_EVENT_LBUTTONDOWN)
  {
    if (params->prev_pt.x >= 0)
    {
      params->points.push_back(params->prev_pt);
      Point pt = Point(x, y);
      line(params->drawImg, params->prev_pt, pt, Scalar(255, 0, 0), 2, 8, 0);
      if (norm(pt - params->points[0]) < params->eps)
      {
        params->points.push_back(pt);
        params->points.push_back(params->points[0]);
        line(params->drawImg, pt, params->points[0], Scalar(255, 0, 0), 2, 8, 0);
        params->prev_pt = Point2f(-1, -1);
        params->binaryMask.setTo(Scalar::all(0));
        Point* ps = new Point[params->points.size()];
        for (size_t i = 0; i < params->points.size(); i++)
        {
          ps[i] = params->points[i];
        }
        fillConvexPoly(params->binaryMask, ps, params->points.size(), Scalar(255));
        delete[] ps;
        params->is_complete = true;
      }
      else
        params->prev_pt = pt;
    }
    else
    {
      params->points.clear();
      params->is_complete = false;
      cvtColor(params->img, params->drawImg, CV_GRAY2RGB);
      params->binaryMask.create(params->img.rows, params->img.cols, CV_8UC1);
      params->binaryMask.setTo(Scalar::all(0));
      params->prev_pt = Point(x, y);
    }
    circle(params->drawImg, Point(x, y), 2, Scalar(255, 0, 0), 2, 8, 0);
    imshow(windName, params->drawImg);
  }
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

void copyCI(string from, string to)
{
  std::ifstream ifs(from.c_str(), std::ios::binary);
  std::ofstream ofs(to.c_str(), std::ios::binary);

  ofs << ifs.rdbuf();
  ifs.close();
  ofs.close();
}

void CropSaver::savePoints()
{
  if (maskParams.is_complete || trainIndex != 1)
  {
    Mat im;
    maskParams.img.copyTo(im);

    char filename[256];
    string config_path = objDir + "/%d.png";
    sprintf(filename, config_path.c_str(), trainIndex);
    imwrite(filename, im);

    vector<Point3f> object_points;

    if (trainIndex == 1)
    {
      for (int u = 0; u < im.rows; ++u)
      {
        for (int v = 0; v < im.cols; ++v)
        {
          if (mask.at<uint16_t> (u, v) != 0 && maskParams.binaryMask.at<unsigned char> (u, v) == 255)
          {
            object_points.push_back(cloudPoints[mask.at<uint16_t> (u, v)]);
          }
        }
      }
    }
    else
    {
      object_points = cloudPoints;
      vector<Point3f> rotated_object_points;
      project3dPoints(cloudPoints, rvecChess, tvecChess, rotated_object_points);

      vector<Point2d> image_points_first_cadr;
      image_points_first_cadr.resize(rotated_object_points.size());
      for (size_t i = 0; i < rotated_object_points.size(); i++)
      {
        cameraInfo.projectPoint(rotated_object_points[i], image_points_first_cadr[i]);
      }
      vector<int> indexes;
      for (size_t i = 0; i < image_points_first_cadr.size(); i++)
      {
        Point2d p = image_points_first_cadr[i];
        bool isPointInMask = (p.x > 0) && (p.y > 0) && (p.x < im.cols) && (p.y < im.rows);
        isPointInMask = isPointInMask && (maskParams.binaryMask.at<unsigned char> (p.y, p.x) == 255);
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
      object_points = temp;
    }

    Mat binMask;
    binMask.create(im.rows, im.cols, CV_8UC1);
    binMask.setTo(Scalar::all(0));
    for (size_t i = 0; i < object_points.size(); ++i)
    {
      cv::Point2f uv;
      cameraInfo.projectPoint(object_points[i], uv);
      int x = (int)uv.x;
      int y = (int)uv.y;
      if (x > 0 && x < im.cols && y > 0 && y < im.rows)
      {
        binMask.at<uint8_t> (y, x) = 255;
      }
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
    for (size_t index = 0; index < object_points.size(); index++)
    {
      file << object_points[index].x << " " << object_points[index].y << " " << object_points[index].z << endl;
    }
    file.close();

    string chess3dpath = objDir + "/chess%d.txt";
    sprintf(filename, chess3dpath.c_str(), trainIndex);
    ofstream chess3dfile;
    chess3dfile.open(filename);
    for (size_t index = 0; index < chess3dPoints.size(); index++)
    {
      chess3dfile << chess3dPoints[index].x << " " << chess3dPoints[index].y << " " << chess3dPoints[index].z << endl;
    }
    chess3dfile.close();

    if (trainIndex != 1)
    {
      config_path = objDir + "/chess%d.xml";
      sprintf(filename, config_path.c_str(), trainIndex);
      FileStorage fs(filename, FileStorage::WRITE);
      fs << "rvec" << rvecChess;
      fs << "tvec" << tvecChess;
      fs.release();
    }
    else
      isSavedFirstImage = true;

    ofstream config((newTrDir + "/config.txt").c_str());//, ios_base::out | ios_base::app);
    printf("Saving to %s\n", (newTrDir + "/config.txt").c_str());
    config << objName << " " << trainIndex;
    config.close();

    trainIndex++;
  }
}

void CropSaver::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  if (!isCamInit)
  {
    saveCameraInfo(ci, ciPath);
    copyCI(ciPath, newTrDir + "/" + objName + "/info.txt");

    cameraInfo.init(ciPath);

    isCamInit = true;

    cout << "Cam is initialized!" << endl;
  }
}

CropSaver::CropSaver(const ros::NodeHandle &_nh, string _cloudTopic, string _imageTopic, string _ciTopic,
                     string _configPath, string _newTrDir, string _objName, bool _isIgnoreRecognize) :
  nh(_nh), queue_size(2000), tr(NULL), sync(SyncPolicy(20)), isCamInit(false)
{
  cloudTopic = _cloudTopic;
  imageTopic = _imageTopic;
  ciTopic = _ciTopic;
  configPath = _configPath;
  newTrDir = _newTrDir;
  objName = _objName;
  isIgnoreRecognize = _isIgnoreRecognize;
  ciPath = newTrDir + "/info.txt";
  isSavedFirstImage = false;
  isTrNeedUpdate = false;
  trainIndex = 1;

  createDirIsNeeded(newTrDir);
  objDir = newTrDir + "/" + objName;
  createDirIsNeeded(objDir);
}

CropSaver::~CropSaver()
{
  if (tr)
    delete tr;
}

void CropSaver::init()
{
  image_sub.subscribe(nh, imageTopic, queue_size);
  cloud_sub.subscribe(nh, cloudTopic, queue_size);
  sync.connectInput(image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&CropSaver::cloudCallback, this, _1, _2));

  ci_sub = nh.subscribe(ciTopic, 1, &CropSaver::cameraInfoCallback, this);

  ros::NodeHandle localnh("~");
  pt_pub = localnh.advertise<visualization_msgs::Marker> ("points", 0);
  image_transport::ImageTransport it(localnh);
  pub = it.advertise("TOD_image_debug", 1);
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

void convertPointCloudToVector(const pcl::PointCloud<pcl::PointXYZ>& cloud, vector<Point3f>& points)
{
  points.clear();

  for (size_t index = 0; index < cloud.points.size(); index++)
  {
    if (!isnan(cloud.points[index].x))
      points.push_back(Point3f(cloud.points[index].x, cloud.points[index].y, cloud.points[index].z));
  }
}

void initTrainingSet(TrainingSet* tr, string ciPath)
{
  assert(tr != NULL);
  tr->isDrawInliers = false;
  tr->isDrawCorrespondence = false;
  tr->isPrintResults = true;
  tr->isDrawProjection = false;
  tr->isDrawClusters = false;
  tr->isNode = false;
  tr->initTestCamModel(ciPath);
}

void CropSaver::cloudCallback(const sensor_msgs::ImageConstPtr& imageMsg,
                              const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  cout << "cloudCallback!" << endl;
  if (!isCamInit)
    return;

  const IplImage* image = bridge.imgMsgToCv(imageMsg, "bgr8");
  IplImage* gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
  cvCvtColor(image, gray, CV_BGR2GRAY);
  Mat(gray).copyTo(maskParams.img);
  Mat(image).copyTo(maskParams.drawImg);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloudMsg, cloud);
  convertPointCloudToVector(cloud, cloudPoints);

  mask.create(maskParams.img.rows, maskParams.img.cols, CV_16U);
  mask.setTo(Scalar::all(0));

  for (size_t i = 0; i < cloudPoints.size(); ++i)
  {
    cv::Point2f uv;
    cameraInfo.projectPoint(cloudPoints[i], uv);
    int x = (int)uv.x;
    int y = (int)uv.y;
    if (x > 0 && x < maskParams.img.cols && y > 0 && y < maskParams.img.rows)
    {
      mask.at<uint16_t> (y, x) = i;
    }
  }

  bool chessFound = false;
  vector<Point2f> chessPointBuf;
  Size boardSize(4, 5);

  findChessboardCorners(maskParams.img, boardSize, chessPointBuf, CV_CALIB_CB_ADAPTIVE_THRESH);
  if (chessPointBuf.size() < boardSize.width * boardSize.height)
  {
    printf("Found %d points, should be %d, skipping...\n", chessPointBuf.size(), boardSize.width * boardSize.height);
    return;
  }

  vector<Point3f> stereoPoints;
  for (size_t indexBuf = 0; indexBuf < chessPointBuf.size(); indexBuf++)
  {
    if (mask.at<uint16_t> (chessPointBuf[indexBuf].y, chessPointBuf[indexBuf].x) != 0)
    {
      stereoPoints.push_back(cloudPoints[mask.at<uint16_t> (chessPointBuf[indexBuf].y, chessPointBuf[indexBuf].x)]);
    }
  }

  if (isSavedFirstImage)
  {
    if (stereoPoints.size() < boardSize.width * boardSize.height)
    {
      cout << "Not enough chessboard points: " << stereoPoints.size() << endl;
      return;
    }

    chess3dPoints.clear();
    copy(stereoPoints.begin(), stereoPoints.end(), back_inserter(chess3dPoints));
    //solvePnP(Mat(stereoPoints), Mat(chessPoints), cameraInfo.K, cameraInfo.D, rvecChess, tvecChess);
    solvePnPRansac(stereoPoints, chessPoints, cameraInfo.K, cameraInfo.D, rvecChess, tvecChess, false, 1000, 5, 20);
    vector<Point3f> rotated_object_points;
    project3dPoints(stereoPoints, rvecChess, tvecChess, rotated_object_points);

    if (firstChessObjectPoints.size() != rotated_object_points.size())
    {
      printf("Skipping, %d chessboard points detected as opposed to %d points on the first frame\n",
             rotated_object_points.size(), firstChessObjectPoints.size());
      return;
    }
    Mat tvecChess1 = calcTranslation(rotated_object_points, firstChessObjectPoints);
    printf("Adjusted translation to %f %f %f\n", tvecChess1.at<double> (0, 0), tvecChess1.at<double> (1, 0),
           tvecChess1.at<double> (2, 0));
    //tvecChess += tvecChess1;

    float error = 0;
    float avg_error = 0;
    vector<Point2f> projected_points;
    projectPoints(Mat(rotated_object_points), Mat(3, 1, CV_64F, Scalar(0.0)), Mat(3, 1, CV_64F, Scalar(0.0)),
                  cameraInfo.K, cameraInfo.D, projected_points);

    cout << "Projected points count " << projected_points.size() << endl;
    for (size_t indexBuf = 0; indexBuf < projected_points.size(); indexBuf++)
    {
      float dist = norm(projected_points[indexBuf] - chessPoints[indexBuf]);
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
      return;
    }
    cout << "Average error = " << sqrt(avg_error / projected_points.size()) << endl;
  }
  else
  {
    firstChessObjectPoints = stereoPoints;
    chessPoints = chessPointBuf;
    maskParams.img.copyTo(firstView);
  }

  bool isRecognized = true;

  if (!isIgnoreRecognize)
    isRecognized = recognizeImage(tr, pt_pub, maskParams.img);

  if (!isRecognized || isIgnoreRecognize)
  {
    if (!isSavedFirstImage)
    {
      imshow(windName, maskParams.drawImg);
      cvSetMouseCallback(windName.c_str(), onMouse, (void*)&maskParams);

      for (;;)
      {
        int c = cvWaitKey(0);
        if ((char)c == 27)
        {
          break;
        }

        if (char(c) == 32)
        {
          maskParams.points.clear();
          maskParams.prev_pt = Point(-1, -1);
          break;
        }

        if ((char)c == 115)
        {
          savePoints();
          isTrNeedUpdate = true;
          break;
        }
      }
    }
    else
    {
      savePoints();
      isTrNeedUpdate = true;
    }

    if (isTrNeedUpdate)
    {
      if (tr != NULL)
        delete tr;

      if (!isIgnoreRecognize)
      {
        tr = new TrainingSet(newTrDir, configPath);
        initTrainingSet(tr, ciPath);
      }

      isTrNeedUpdate = false;
    }
  }
}

int main(int argc, char **argv)
{
  if (argc < 6)
  {
    printf("This is a tool for creating training base.\n"
      "Usage: crop\n"
      "     [-config <file.config>]    # configuration file with algorithm parameters\n"
      "     [-ignore-recognizing]      # ignore result of recognize function\n"
      "     <ci_topic>                 # parameters of camera\n"
      "     <cloud_topic>              # name of topic with points cloud\n"
      "     <image_topic>              # name of topic with image\n"
      "     <object_name>              # object name in training base\n"
      "     <train_dir>                # folder for new training base\n"
      "\n");
    return 1;
  }

  namedWindow(windName, 1);

  string cloudTopic = "", imageTopic = "", configPath = "", newTrDir = "", ciTopic = "", objName = "";
  bool isIgnoreRecognize = false;
  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else if (strcmp(s, "-ignore-recognizing") == 0)
    {
      isIgnoreRecognize = true;
    }
    else
    {
      ciTopic = string(argv[i++]);
      cloudTopic = string(argv[i++]);
      imageTopic = string(argv[i++]);
      objName = string(argv[i++]);
      newTrDir = string(argv[i]);
    }
  }

  ros::init(argc, argv, "saver");
  ros::NodeHandle _nh;
  CropSaver node(_nh, cloudTopic, imageTopic, ciTopic, configPath, newTrDir, objName, isIgnoreRecognize);
  node.init();

  ros::spin();
  return 0;
}

