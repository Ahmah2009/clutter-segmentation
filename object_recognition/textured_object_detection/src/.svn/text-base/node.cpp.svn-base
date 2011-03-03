#include <visualization_msgs/Marker.h>
#include "opencv/highgui.h"
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <textured_object_detection/training_set.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

using namespace cv;

string winName = "image";
sensor_msgs::CvBridge bridge;
TrainingSet* tr;
ros::Publisher pt_pub;
std::vector <ObjectInfo> objects;

image_transport::Publisher pub_proj;
image_transport::Publisher pub_bb;
int file_count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
  tr->setTestCameraInfo(cameraInfo);

  const IplImage* image = bridge.imgMsgToCv(msg, "bgr8");
  IplImage* gray = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
  cvCvtColor(image, gray, CV_RGB2GRAY);

  Mat img(gray);
  tr->recognize(img, objects, pt_pub);
  for (size_t i = 0; i < objects.size(); i++)
    ROS_INFO("Object's id - %d, object's name - %s\n", objects[i].objectId, objects[i].objectName.c_str());
  if (objects.size() == 0)
    ROS_INFO("Objects are not found!\n");
  cvReleaseImage(&gray);

//  imshow(winName, img);
//  waitKey(100);
#if 0
  char buf[1024];
  sprintf(buf, "/u/ve/data/binpick/temp/frame%04d.jpg", file_count++);
  printf("Writing to %s\n", buf);
  imwrite(buf, tr->projImg);
#endif

  Mat projImg;
  resize(tr->projImg, projImg, Size(tr->projImg.cols*tr->params.scale, tr->projImg.rows*tr->params.scale));

  sensor_msgs::Image img_msg;
  fillImage(img_msg, sensor_msgs::image_encodings::BGR8, projImg.rows, projImg.cols, projImg.step, const_cast<uint8_t*>(projImg.data));
  pub_proj.publish(img_msg);

  sensor_msgs::Image img_msg_bb;
  fillImage(img_msg, sensor_msgs::image_encodings::BGR8, tr->bbProjImg.rows, tr->bbProjImg.cols, tr->bbProjImg.step, const_cast<uint8_t*>(tr->bbProjImg.data));
  pub_bb.publish(img_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tod_detector");
  ros::NodeHandle localnh("~");
  ros::NodeHandle nh;
  pt_pub = localnh.advertise<visualization_msgs::Marker> ("points", 0);
      
//  namedWindow(winName);
//  cvStartWindowThread();
  
  std::string base_dir, configPath;
  nh.getParam("training_base_dir", base_dir);
  nh.getParam("config_file", configPath);
  printf("Reading config path %s\n", configPath.c_str());
  tr = new TrainingSet(base_dir, configPath);
  tr->isDrawInliers = false;
  tr->isDrawCorrespondence = false;
  tr->isPrintResults = true;
  tr->isDrawProjection = true;
  tr->isDrawClusters = false;
  tr->isNode = true;
  string paramsPath = base_dir + "/info.txt";
  tr->initTestCamModel(paramsPath);
   
  image_transport::ImageTransport it(localnh);
  string img_topic = nh.resolveName("image_in");
  image_transport::CameraSubscriber cam_sub = it.subscribeCamera(img_topic, 1, imageCallback);
  
  pub_proj = it.advertise("image_proj", 1);
  pub_bb = it.advertise("image_bb", 1);

  ros::spin();
}

