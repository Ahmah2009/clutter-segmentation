#include "textured_object_detection/training_set.h"

using namespace std;

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    printf(
           "This is a matching sample.\n"
             "Usage: ms\n"
             "     [-config <file.config>]    # configuration file with algorithm parameters\n"
             "     <train_image>              # point cloud file with same name and camera info file should be in the directory, assumed that test image was taken with camera with same parameters\n"
             "     <test_image>               # test image\n"
             "\n");
    return 1;
  }

  string configPath = "", trainImagePath = "", testImagePath = "";

  for (int i = 1; i < argc; i++)
  {
    const char* s = argv[i];
    if (strcmp(s, "-config") == 0)
    {
      configPath = string(argv[++i]);
    }
    else
    {
      trainImagePath = string(argv[i++]);
      testImagePath = string(argv[i]);
    }
  }

  TrainingSet tr("", configPath);
  tr.drawInliers(testImagePath, trainImagePath);
  return 0;
}
