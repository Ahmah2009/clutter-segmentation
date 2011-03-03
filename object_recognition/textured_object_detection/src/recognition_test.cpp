#include "opencv/highgui.h"
#include <ros/ros.h>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <visualization_msgs/Marker.h>
#include "textured_object_detection/training_set.h"

using namespace cv;
using namespace std;

#define CONFIGFILE "config.txt"
int imgscount = -1;
string testCamFile;
double allTime = 0.0;
TrainingSet* tr;
bool isWriteLog = false;
int testIndex = 1;
Ptr<FileStorage> fs;

void calculate(const string& dir, const ros::Publisher &pt_pub)
{
  if (isWriteLog)
  {
    stringstream out;
    out << testIndex;
    testIndex++;
    (*fs) << "test" + out.str() << "{";
    (*fs) << "testFolder" << dir;
    (*fs) << "objects" << "{";
  }

  int totalObjectsCount = 0;

  int ind = 0;
  while(ind < imgscount || imgscount == -1)
  {
    ostringstream oss;
    oss << (ind + 1);
    string path = dir + oss.str() + ".png";
    cout << "< Reading the image... " << path;
    Mat img = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    cout << ">" << endl;
    if (img.empty())
    {
      cout << "Can not read test image" << endl;
      break;
    }

    vector<ObjectInfo> objects;
    cout << "Recognizing..." << endl;
    double t = (double)getTickCount();
    tr->recognize(img, objects, pt_pub);
    t = ((double)getTickCount() - t);
    allTime += t;
    t = t / getTickFrequency();
    cout << "All time = " << t << endl;
    for (size_t objId = 0; objId < objects.size(); objId++)
      cout << "Object is " << objects[objId].objectId << endl;
    if (objects.size() == 0)
      cout << "Objects are not found!" << endl;

    if (isWriteLog)
    {
      ObjectInfo::saveObjects(*fs, ind+1, objects, totalObjectsCount);
    }

    ind++;
  }

  if (isWriteLog)
  {
    (*fs) << "objectsCount" << totalObjectsCount << "}" << "}";
  }
}

bool initTestCamParametersFilePath(string trainDir, string objectTestDir)
{
  string path = objectTestDir + "/info.txt";
  ifstream camFile;
  camFile.open(path.c_str());
  if (camFile.is_open() == false)
  {
    path = trainDir + "/info.txt";
    ifstream camFileFromTrain;
    camFileFromTrain.open(path.c_str());
    if (camFileFromTrain.is_open() == false)
      return false;
    else
    {
      camFileFromTrain.close();
      testCamFile = path;
      return true;
    }
  }
  else
  {
    camFile.close();
    testCamFile = path;
    return true;
  }
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    printf(
           "This is a recognition test.\n"
             "Usage: rt\n"
             "     [-config <file.config>]    # configuration file with algorithm parameters\n"
             "     [-log <log.xml>]           # path to log file\n"
             "     [-count count]             # images count in each folder\n"
             "     <train_folder>             # folder contained training base\n"
             "     <test_folder>              # test base\n"
             "\n");
    return 1;
  }

  string configPath = "", trainFolder = "", testFolder = "", logPath = "";

  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else  if (strcmp(s, "-log") == 0)
    {
      logPath = string(argv[++i]);
      isWriteLog = true;
      fs = new FileStorage(logPath, FileStorage::WRITE);
      if (!fs->isOpened())
      {
        cout << "Can't create log file!" << endl;
        return -1;
      }
    }
    else  if (strcmp(s, "-count") == 0)
    {
      imgscount = atoi(argv[++i]);
    }
    else
    {
      trainFolder = string(argv[i++]);
      testFolder = string(argv[i]);
    }
  }

  if (isWriteLog)
  {
    (*fs) << "trainFolder" << trainFolder;
  }

  ros::init(argc, argv, "detector");
  ros::NodeHandle nh("~");
  ros::Publisher pt_pub = nh.advertise<visualization_msgs::Marker> ("points", 0);

  cout << "< Reading train base... " << endl;
  cout << flush;
  tr = new TrainingSet(trainFolder, configPath);
  tr->isDrawInliers = false; // draw inliers after successful matches
  tr->isDrawCorrespondence = false; // draw point matches to all dB objects from scene
  tr->isPrintResults = false;
  tr->isDrawProjection = false;
  tr->isDrawClusters = false;
  cout << ">" << endl;

  ifstream file;
  string filepath = string(testFolder) + "/" + CONFIGFILE;
  file.open(filepath.c_str());
  int objCount = 0;
  if (file.is_open() != false)
  {
    string name;
    while (!file.eof())
    {
      file >> name;
      objCount++;
      string dir = testFolder;
      dir += "/" + name + "/";
      if (!initTestCamParametersFilePath(trainFolder, dir))
      {
        cout << "Can't open file with camera parameters for test image!" << endl;
        return 1;
      }
      else
      {
        tr->initTestCamModel(testCamFile);
      }
      calculate(dir, pt_pub);
    }
  }
  else
  {
    struct dirent* dent;
    DIR* srcdir = opendir(testFolder.c_str());
    if (srcdir == NULL)
    {
      return -1;
    }
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
        objCount++;
        string dir = testFolder;
        cout << "Name = " << string(dent->d_name) << endl;
        dir += "/" + string(dent->d_name) + "/";
        if (!initTestCamParametersFilePath(trainFolder, dir))
        {
          cout << "Can't open file with camera parameters for test image!" << endl;
          return 1;
        }
        else
        {
          tr->initTestCamModel(testCamFile);
        }
        calculate(dir, pt_pub);
      }
    }
    closedir(srcdir);
  }
  allTime = allTime / getTickFrequency();
  cout << "Time = " << allTime << " " << allTime / (float)(objCount * imgscount) << endl;
  delete tr;

  fs->release();

  return 0;
}
