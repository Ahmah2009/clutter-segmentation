#include "opencv/highgui.h"
#include <fstream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include "textured_object_detection/training_set.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  if( argc < 3 )
  {
    printf( "This is a recognition sample.\n"
        "Usage: rs\n"
        "     [-ci <testCameraInfo.txt>] # parameters of test camera\n"
        "     [-config <file.config>]    # configuration file with algorithm parameters\n"
        "     <train_folder>             # folder with training base\n"
        "     <test_image>               # test image\n"
        "\n" );
    return 1;
  }

  string testCiPath = "", configPath = "", trainFolder = "", testImagePath = "";

  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-ci") == 0)
    {
      testCiPath = string(argv[++i]);
    }
    else if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else
    {
      trainFolder = string(argv[i++]);
      testImagePath = string(argv[i]);
    }
  }

  ros::init(argc, argv, "detector");
  ros::NodeHandle nh("~");
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker> ("points", 0);

  cout << "< Reading train base... ";
  TrainingSet tr(trainFolder, configPath);
  tr.isDrawInliers = false;
  tr.isDrawCorrespondence = false;
  tr.isPrintResults = true;
  tr.isDrawProjection = true;
  tr.isDrawClusters = false;
  tr.isNode = false;

  string testCamFile;
  if (testCiPath.empty())
    testCamFile = trainFolder + "/info.txt";
  else
    testCamFile = testCiPath;
  ifstream camFile;
  camFile.open(testCamFile.c_str());
  if (camFile.is_open() == false)
  {
    cout << "Can't open file with camera parameters for test image!" << endl;
    return 1;
  }
  else
    camFile.close();

  tr.initTestCamModel(testCamFile);
  cout << ">" << endl;

  ostringstream oss;
  Mat img = imread(testImagePath, CV_LOAD_IMAGE_GRAYSCALE);
  if (img.empty())
  {
    cout << "Can not read test image" << endl;
    return -1;
  }
  cout << "Recognizing..." << endl;
  double t = (double)getTickCount();
  vector<ObjectInfo> objects;
  tr.recognize(img, objects, pt_pub);
  t = ((double)getTickCount() - t) / getTickFrequency();
  cout << "All time = " << t << endl;
  for (size_t i = 0; i < objects.size(); i++)
    cout << "Object's id - " << objects[i].objectId << ", object's name - " << objects[i].objectName << endl;
  if (objects.size() == 0)
    cout << "Objects are not found!" << endl;

  const bool saveObjects = false;
  if(saveObjects)
  {
    string resultsFileName = testImagePath.substr(0, testImagePath.find_last_of('.') + 1) + "xml";
    FileStorage fs(resultsFileName, FileStorage::WRITE);
    ObjectInfo::saveObjects(fs, objects);
    cout << "Writing results to " << resultsFileName << endl;
  }
  return 0;
}
