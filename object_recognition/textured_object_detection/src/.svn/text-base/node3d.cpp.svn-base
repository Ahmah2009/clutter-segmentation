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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class TrainingSet3dNode
{
public:
  TrainingSet3dNode(const ros::NodeHandle &nh);
  virtual ~TrainingSet3dNode();
  void init();
private:
  void proCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void imageCallback(const sensor_msgs::ImageConstPtr& pro_msg, const sensor_msgs::ImageConstPtr& l_msg,
                                        const sensor_msgs::ImageConstPtr& r_msg);
  void initCameraModel(string filename);
  void leftCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void rightCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);

  sensor_msgs::CvBridge bridge;
  sensor_msgs::CvBridge lbridge;
  sensor_msgs::CvBridge rbridge;
  ros::Publisher pt_pub;
  vector <ObjectInfo> objects;
  image_transport::Publisher pub;
  ros::Subscriber ci_sub_l, ci_sub_r;

  Subscriber<sensor_msgs::Image> image_sub_pro;
  Subscriber<sensor_msgs::Image> image_sub_l;
  Subscriber<sensor_msgs::Image> image_sub_r;
  message_filters::Synchronizer<SyncPolicy> image_sync;
  int queue_size;

  bool isLeftCamInit, isRightCamInit;
  CameraInfo leftInfo, rightInfo;

  TrainingSet3d* tr;
  image_geometry::PinholeCameraModel* cm;

  Mat rvec, tvec;

  ros::NodeHandle nh;
};

void TrainingSet3dNode::leftCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  if (!isLeftCamInit)
  {
    leftInfo.height = ci->height;
    leftInfo.width = ci->width;
    for (int i = 0; i < 5; i++)
      leftInfo.D_values[i] = ci->D[i];
    for (int i = 0; i < 9; i++)
      leftInfo.K_values[i] = ci->K[i];
    for (int i = 0; i < 9; i++)
      leftInfo.R_values[i] = ci->R[i];
    for (int i = 0; i < 12; i++)
      leftInfo.P_values[i] = ci->P[i];
    isLeftCamInit = true;
  }
}

void TrainingSet3dNode::rightCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  if (!isRightCamInit)
  {
    rightInfo.height = ci->height;
    rightInfo.width = ci->width;
    for (int i = 0; i < 5; i++)
      rightInfo.D_values[i] = ci->D[i];
    for (int i = 0; i < 9; i++)
      rightInfo.K_values[i] = ci->K[i];
    for (int i = 0; i < 9; i++)
      rightInfo.R_values[i] = ci->R[i];
    for (int i = 0; i < 12; i++)
      rightInfo.P_values[i] = ci->P[i];
    isRightCamInit = true;
  }
}

TrainingSet3dNode::TrainingSet3dNode(const ros::NodeHandle &_nh) :
  nh(_nh), queue_size(2000) ,tr(NULL), cm(NULL), image_sync(SyncPolicy(20)), isRightCamInit(false), isLeftCamInit(false)
{
}

TrainingSet3dNode::~TrainingSet3dNode()
{
  if (tr)
    delete tr;
  if (cm)
      delete cm;
}

void TrainingSet3dNode::initCameraModel(string filename)
{
  ifstream file;
  file.open(filename.c_str());
  sensor_msgs::CameraInfo ci;
  file >> ci.height >> ci.width;
  for (int i = 0; i < 5; i++)
    file >> ci.D[i];
  for (int i = 0; i < 9; i++)
    file >> ci.K[i];
  for (int i = 0; i < 9; i++)
    file >> ci.R[i];
  for (int i = 0; i < 12; i++)
    file >> ci.P[i];
  file.close();

  cm = new image_geometry::PinholeCameraModel();
  cm->fromCameraInfo(ci);
}

void TrainingSet3dNode::init()
{
  string imageTopic, imagelTopic, imagerTopic;
  nh.getParam("image_in", imageTopic);
  nh.getParam("imagel_in", imagelTopic);
  nh.getParam("imager_in", imagerTopic);
  image_sub_pro.subscribe(nh, imageTopic, queue_size);
  image_sub_l.subscribe(nh, imagelTopic, queue_size);
  image_sub_r.subscribe(nh, imagerTopic, queue_size);
  image_sync.connectInput(image_sub_pro, image_sub_l, image_sub_r);
  image_sync.registerCallback(boost::bind(&TrainingSet3dNode::imageCallback, this, _1, _2, _3));

  ci_sub_l = nh.subscribe("/narrow_stereo_textured/left/camera_info", 1, &TrainingSet3dNode::leftCameraInfoCallback, this);
  ci_sub_r = nh.subscribe("/narrow_stereo_textured/right/camera_info", 1, &TrainingSet3dNode::rightCameraInfoCallback, this);

  ros::NodeHandle localnh("~");
  pt_pub = localnh.advertise<visualization_msgs::Marker> ("points", 0);
  image_transport::ImageTransport it(localnh);
  pub = it.advertise("TOD_image_debug", 1);

  std::string base_dir, configPath;
  nh.getParam("training_base_dir", base_dir);
  nh.getParam("config_file", configPath);
  tr = new TrainingSet3d(base_dir, configPath);
  tr->isDrawInliers = false;
  tr->isDrawCorrespondence = false;
  tr->isPrintResults = true;
  tr->isDrawProjection = true;
  tr->isDrawClusters = false;
  tr->isNode = true;
  string paramsPath = base_dir + "/info.txt";
  initCameraModel(paramsPath);
  tr->initTestCamModel(paramsPath);

  std::string transformPath;
  nh.getParam("transform_in", transformPath);
  FileStorage fs;
  fs.open(transformPath.c_str(), FileStorage::READ);
  if (fs.isOpened()){
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;
    fs.release();
  }
  else
  {
    ROS_ERROR("TrainingSet3dNode::init: Could not open transform file");
    ros::shutdown();
  }
}

void TrainingSet3dNode::imageCallback(const sensor_msgs::ImageConstPtr& pro_msg, const sensor_msgs::ImageConstPtr& l_msg,
                                      const sensor_msgs::ImageConstPtr& r_msg)
{
  assert (cm != NULL);
  if (!isLeftCamInit || !isRightCamInit)
    return;
  const IplImage* image = bridge.imgMsgToCv(pro_msg, "bgr8");
  IplImage* gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
  cvCvtColor(image, gray, CV_BGR2GRAY);
  Mat img(gray);

  const IplImage* imagel = bridge.imgMsgToCv(l_msg, "bgr8");
  IplImage* grayl = cvCreateImage(cvSize(imagel->width, imagel->height), IPL_DEPTH_8U, 1);
  cvCvtColor(imagel, grayl, CV_BGR2GRAY);
  Mat imgl(grayl);

  const IplImage* imager = bridge.imgMsgToCv(r_msg, "bgr8");
  IplImage* grayr = cvCreateImage(cvSize(imager->width, imager->height), IPL_DEPTH_8U, 1);
  cvCvtColor(imager, grayr, CV_BGR2GRAY);
  Mat imgr(grayr);

  Mat leftr, rightr;
  imgl.copyTo(leftr);
  leftInfo.rectify(imgl, leftr);

  imgr.copyTo(rightr);
  leftInfo.rectify(imgr, rightr);


  Mat d, tmp;
  StereoBM(StereoBM::BASIC_PRESET, 128, 15)(leftr, rightr, d);
  d.convertTo(tmp, CV_8U, 1.0 / 16.0);

  Mat_<double> Q(4, 4, 0.0);
  initQ(Q, rightInfo);
  Mat_<Vec3f> point_cloud;
  reprojectImageTo3D(d, point_cloud, Q, true);

  vector<Point3f> points;
  for (int u = 0; u < point_cloud.rows; ++u)
  {
    for (int v = 0; v < point_cloud.cols; ++v)
    {
      if (isValidPoint(point_cloud.at<Vec3f> (u, v)))
      {
        points.push_back(Point3f(point_cloud.at<Vec3f> (u, v)[0], point_cloud.at<Vec3f> (u, v)[1],
                                 point_cloud.at<Vec3f> (u, v)[2]));
      }
    }
  }


  vector<Point3f> rotated_object_points;
  project3dPoints(points, rvec, tvec, rotated_object_points);

  Mat maskImage;
  maskImage.create(img.rows, img.cols, CV_16UC1);
  maskImage.setTo(Scalar::all(0));
  for (size_t i = 0; i < rotated_object_points.size(); ++i)
  {
    Point3d pt_cv = rotated_object_points[i];
    Point2d uv;
    cm->project3dToPixel(pt_cv, uv);
    int x = (int)uv.x;
    int y = (int)uv.y;
    int distantion = 5;
    if (x > 0 && x < img.cols && y > 0 && y < img.rows)
    {
      for (int xx = x - distantion; xx < x + distantion && xx > 0 && xx < img.cols; xx++)
        for (int yy = y - distantion; yy < y + distantion && yy > 0 && yy < img.rows; yy++)
          maskImage.at<uint16_t> (yy, xx) = i + 1;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> resultCloud;
  for (size_t pointsIndex = 0; pointsIndex < rotated_object_points.size(); pointsIndex++)
  {
    pcl::PointXYZ p;
    p.x = rotated_object_points[pointsIndex].x;
    p.y = rotated_object_points[pointsIndex].y;
    p.z = rotated_object_points[pointsIndex].z;
    resultCloud.points.push_back(p);
  }

  tr->recognize(img, maskImage, resultCloud, objects, pt_pub);

  for (size_t i = 0; i < objects.size(); i++)
    ROS_INFO("Object's id - %d, object's name - %s\n", objects[i].objectId, objects[i].objectName.c_str());
  if (objects.size() == 0)
    ROS_INFO("Objects are not found!\n");
  cvReleaseImage(&gray);

  sensor_msgs::Image img_msg;
  fillImage(img_msg, sensor_msgs::image_encodings::BGR8, tr->projImg.rows, tr->projImg.cols, tr->projImg.step, const_cast<uint8_t*>(tr->projImg.data));
  pub.publish(img_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recognizer");
  ros::NodeHandle _nh;

  TrainingSet3dNode node(_nh);
  node.init();

  ros::spin();
  return 0;
}

