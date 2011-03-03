#include "opencv/highgui.h"

#include "textured_object_detection/training_set.h"
#include "textured_object_detection/shared_functions.h"

#include <visualization_msgs/Marker.h>
#include "posest/pnp_ransac.h"

#include <iostream>

using namespace std;
using namespace cv;

struct CroppingAutoParameters
{
public:
  string configPath, trasformPath;
  int skippedFrames;
  bool isCalculateMaskOnly;
  bool isIgnoreRecognizing;

  string sourceFolder;

  string objectName;
  string trainingDirectory;

  CroppingAutoParameters(): configPath(""), trasformPath(""), skippedFrames(0), isCalculateMaskOnly(false),
      isIgnoreRecognizing(false), sourceFolder(""), objectName(""), trainingDirectory("")
  {}

  void printPrompt(const int& argc)
  {
    if (argc < 4)
      cout << "Tool for creating training base.\n"
        "Usage: crop_auto\n"
        "     [-config <file.config>]    # configuration file with algorithm parameters\n"
        "     [-transform <transform.xml>] # transform from stereo to prosilica\n"
        "     [-skip-frames <number>]    # skip <number> frames\n"
        "     [-mask-only]               # label first frame mask only\n"
        "     [-ignore-recognizing]      # ignore result of recognize function\n"
        "     <src_directory>            # should contain left and right subfolders\n"
        "     <object_name>              # object name in training base\n"
        "     <train_dir>                # folder for new training base\n"
        << endl;
  }

  bool fillParamerers(const int& argc, const char* argv[])
  {
    try
    {
      for (int argIndex = 1; argIndex < argc; argIndex++)
      {
        const char* s = argv[argIndex];
        if (strcmp(s, "-config") == 0)
        {
          configPath = string(argv[++argIndex]);
        }
        else if (strcmp(s, "-transform") == 0)
        {
          trasformPath = string(argv[++argIndex]);
        }
        else if (strcmp(s, "-skip-frames") == 0)
        {
          skippedFrames = atoi(argv[++argIndex]);
        }
        else if (strcmp(s, "-mask-only") == 0)
        {
          isCalculateMaskOnly = true;
        }
        else if (strcmp(s, "-ignore-recognizing") == 0)
        {
          isIgnoreRecognizing = true;
        }
        else if (strncmp(s, "-", 1) == 0)
        {
          cout << "Unsupported argument: " << s << endl;
          return false;
        }
        else
        {
          sourceFolder = string(argv[argIndex++]);
          objectName = string(argv[argIndex++]);
          trainingDirectory = string(argv[argIndex]);
        }
      }
      return true;
    }
    catch (...)
    {
      return false;
    }
  }
};

class StereoCalculator
{
public:
  CameraInfo leftCameraInfo, rightCameraInfo;

  StereoCalculator(const string leftCameraInfoPath, const string rightCameraInfoPath)
  {
    leftCameraInfo.init(leftCameraInfoPath);
    rightCameraInfo.init(rightCameraInfoPath);
  }

  bool isPixelInMask(const Mat& mask, int rowIndex, int colIndex)
  {
    bool result;

    if (mask.empty())
     result = true;
    else
    {
      result = (mask.at<unsigned char> (rowIndex, colIndex) == 255);
    }

    return result;
  }

  void calculateStereo(const Mat& leftImage, const Mat& rightImage, vector<Point3f>& calculatedCloud,
                       Mat& convertedDisparity, const Mat& mask = Mat())
  {
    calculatedCloud.clear();

    Mat leftRectifiedImage, rightRectifiedImage;
    leftCameraInfo.rectify(leftImage, leftRectifiedImage);
    rightCameraInfo.rectify(rightImage, rightRectifiedImage);

    Mat disparity;
    StereoBM(StereoBM::BASIC_PRESET, 128, 15)(leftRectifiedImage, rightRectifiedImage, disparity);
    disparity.convertTo(convertedDisparity, CV_8U, 1.0 / 16.0);

    Mat pointsCloud;
    Mat_<double> Q(4, 4, 0.0);
    initQ(Q, rightCameraInfo);
    reprojectImageTo3D(disparity, pointsCloud, Q, true);

    calculatedCloud.reserve(pointsCloud.rows * pointsCloud.cols);

    for (int rowIndex = 0; rowIndex < pointsCloud.rows; ++rowIndex)
    {
      for (int colIndex = 0; colIndex < pointsCloud.cols; ++colIndex)
      {
        if (isValidPoint(pointsCloud.at<Vec3f>(rowIndex, colIndex)) && isPixelInMask(mask, rowIndex, colIndex))
        {
          Vec3f point = pointsCloud.at<Vec3f> (rowIndex, colIndex);
          calculatedCloud.push_back(Point3f(point[0], point[1], point[2]));
        }
      }
    }
  }
};


class SourceReader
{
public:
  CroppingAutoParameters params;

  virtual bool readImageAndCloud(Mat& image, vector<Point3f>& cloud, void* additionalData = NULL) = 0;
};

class SourceReaderForFiles: public SourceReader
{
public:
  int fileIndex;

  SourceReaderForFiles(const CroppingAutoParameters& parameters)
  {
    params = parameters;

    if (params.skippedFrames)
      fileIndex = params.skippedFrames;
    else
      fileIndex = 1;
  }

  enum ImageType {LEFT_TEX = 0, RIGHT_TEX, LEFT, RIGHT, PROSILICA};

  string getPathForImage(const ImageType& type, const int& index)
  {
    string config_str = params.sourceFolder + "/%s/%d.png";
    char filename[256];
    filename[0] = '\0';

    switch (type)
    {
      case LEFT:
      {
        sprintf(filename, config_str.c_str(), "left", index);
      }
      case RIGHT:
      {
        sprintf(filename, config_str.c_str(), "right", index);
      }
      case LEFT_TEX:
      {
        sprintf(filename, config_str.c_str(), "left_tex", index);
      }
      case RIGHT_TEX:
      {
        sprintf(filename, config_str.c_str(), "right_tex", index);
      }
      case PROSILICA:
      {
        sprintf(filename, config_str.c_str(), "pro", index);
      }
    }
    return string(filename);
  }

  enum CameraType {LEFT_CAM = 0, RIGHT_CAM, PROSILICA_CAM};

  string getPathForCameraInfo(CameraType type)
  {
    switch (type)
    {
      case LEFT_CAM:
      {
        return params.sourceFolder + "/left_info.txt";
      }
      case RIGHT_CAM:
      {
        return params.sourceFolder + "/right_info.txt";
      }
      case PROSILICA_CAM:
      {
        return params.sourceFolder + "/pro_info.txt";
      }
    }
    return "";
  }



  virtual bool readImageAndCloud(Mat& image, vector<Point3f>& cloud, void* additionalData = NULL)
  {
    Mat leftTex = imread(getPathForImage(LEFT_TEX, fileIndex), 0);
    Mat rightTex = imread(getPathForImage(RIGHT_TEX, fileIndex), 0);
  }
};

int main (int argc, const char *argv[])
{
  CroppingAutoParameters params;
  params.printPrompt(argc);

  if (!params.fillParamerers(argc, argv))
  {
    cout << "Can't parse command line parameters." << endl;
    return -1;
  }

  return 0;
}
