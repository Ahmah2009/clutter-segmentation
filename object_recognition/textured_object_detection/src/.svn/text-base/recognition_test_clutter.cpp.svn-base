#include "opencv/highgui.h"
#include <fstream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "textured_object_detection/training_set.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  srand(0);

  if (argc < 3)
  {
    printf("This is a recognition sample for few test images.\n"
      "Usage: rtc\n"
      "     [-ci <testCameraInfo.txt>] # parameters of test camera\n"
      "     [-config <file.config>]    # configuration file with algorithm parameters\n"
      "     [-log <log.xml>]           # path to log file\n"
      "     [-count <count>]           # count of images (using images from '1.png' to '<count>.png')\n"
      "     <train_folder>             # folder with training base\n"
      "     <test_folder>              # folder with test images (format of name #.png)\n"
      "\n");
    return 1;
  }

  string testCiPath = "", configPath = "", trainFolder = "", testFolder = "", logPath = "";
  int count = -1;

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
    else if (strcmp(s, "-count") == 0)
    {
      count = atoi(argv[++i]);
    }
    else if (strcmp(s, "-log") == 0)
    {
      logPath = string(argv[++i]);
    }
    else
    {
      trainFolder = string(argv[i++]);
      testFolder = string(argv[i]);
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
  tr.isDrawProjection = false;
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

  FileStorage fs;
  int totalObjectsCount = 0;

  if (!logPath.empty())
  {
    int testIndex = 1;
    bool isLogExist = true;
    fs.open(logPath, FileStorage::READ);
    if (fs.isOpened())
    {
      while(true)
      {
        FileNode testNode;
        stringstream out;
        out << testIndex;
        testNode = fs["test"+out.str()];
        if (testNode.empty())
        {
          break;
        }
        else
        {
          testIndex++;
        }
      }
    }
    else
    {
      isLogExist = false;
    }
    fs.release();

    fs.open(logPath, FileStorage::APPEND);
    stringstream out;
    out << testIndex;
    if (!isLogExist)
    {
      fs << "trainFolder" << trainFolder;
    }
    fs << "test" + out.str() << "{";
    fs << "testFolder" << testFolder;
    fs << "objects" << "{";
  }

  int ind = 0;
  while(ind < count || count == -1)
  {
    ostringstream oss;
    oss << (ind + 1);
    string path = testFolder + "/" + oss.str() + ".png";
    cout << "< Reading the image... " << path;
    Mat img = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    cout << ">" << endl;
    if (img.empty())
    {
      cout << "Can not read test image" << endl;
      break;
    }

#if 0
    // switching on saving of files
    tr.isSaveImages = true;
    tr.testName = path.substr(0, path.find_last_of('.'));
#endif

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


    if (!logPath.empty())
    {
      ObjectInfo::saveObjects(fs, ind+1, objects, totalObjectsCount);
    }
    ind++;
  }

  if (!logPath.empty())
  {
    fs << "objectsCount" << totalObjectsCount << "}" << "}";
    fs.release();
  }

  return 0;
}
