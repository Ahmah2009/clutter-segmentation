#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <numeric>
#include <algorithm>

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("This is pose correctness analyzer.\n"
      "Usage: log\n"
      "     <result.xml>                  # log file with results of stat working"
      "\n");
    return 1;
  }

  FileStorage fs(argv[1], FileStorage::READ);
  int testIndex = 1;
  while (true)
  {
    FileNode testNode;
    stringstream out;
    out << testIndex;
    string testNodeName = "test" + out.str();
    testNode = fs[testNodeName];
    if (testNode.empty())
    {
      break;
    }
    else
    {
      string testFolder = "";
      testNode["testFolder"] >> testFolder;
      cout << endl << endl << testNodeName << endl;
      cout << testFolder << endl << endl;

      int inliersPosesCount = 0;
      testNode["allInliersCount"] >> inliersPosesCount;
      char nodeName[256];
      vector<Mat> clouds;
      vector<double> planeErrors, totalErrors, zErrors, distances, sizes;
      clouds.resize(inliersPosesCount);

      planeErrors.resize(inliersPosesCount);
      totalErrors.resize(inliersPosesCount);
      zErrors.resize(inliersPosesCount);
      distances.resize(inliersPosesCount);
      sizes.resize(inliersPosesCount);

      for (int i = 1; i <= inliersPosesCount; i++)
      {
        sprintf(nodeName, "object%d", i);
        FileNode tm = testNode[const_cast<const char*>(nodeName)];
        tm["cloud"] >> clouds[i - 1];
        tm["distanceToCam"] >> distances[i - 1];
        tm["size"] >> sizes[i - 1];
      }

      if (inliersPosesCount > 0)
      {
        Mat avg;
        clouds[0].copyTo(avg);
        for (int i = 1; i < inliersPosesCount; i++)
        {
          avg += clouds[i];
        }

        avg /= inliersPosesCount;

        cout << "Pose ID" << "\t" << "Total error" << "\t" << "Plane error" << "\t" << "Z error" << "\t"
            << "Distance to cam" << "\t" << "Object's size" << endl;

        for (int i = 0; i < inliersPosesCount; i++)
        {

          Mat diff = clouds[i] - avg;
          double totalError = 0.0;
          double planeError = 0.0, zError = 0.0;
          for (int j = 0; j < diff.rows; j++)
          {
            totalError += norm(diff.at<Point3f> (j, 0));

            planeError += norm(Point2f(diff.at<Point3f> (j, 0).x, diff.at<Point3f> (j, 0).y));

            zError += fabs(diff.at<Point3f> (j, 0).z);
          }
          totalErrors[i] = totalError / diff.rows;
          planeErrors[i] = planeError / diff.rows;
          zErrors[i] = zError / diff.rows;
          cout << i + 1 << "\t" << totalErrors[i] << "\t" << planeErrors[i] << "\t" << zErrors[i] << "\t"
              << distances[i] << "\t" << sizes[i] << endl;
        }

        double sumTotalError = accumulate(totalErrors.begin(), totalErrors.end(), 0.0);
        double sumPlaneError = accumulate(planeErrors.begin(), planeErrors.end(), 0.0);
        double sumZerror = accumulate(zErrors.begin(), zErrors.end(), 0.0);
        cout << "Average total error = " << sumTotalError / totalErrors.size() << endl;
        cout << "Average plane error = " << sumPlaneError / planeErrors.size() << endl;
        cout << "Average Z error =     " << sumZerror / zErrors.size() << endl;
        cout << "Maximum total error = " << *max_element(totalErrors.begin(), totalErrors.end()) << endl;
        cout << "Maximum plane error = " << *max_element(planeErrors.begin(), planeErrors.end()) << endl;
        cout << "Maximum Z error =     " << *max_element(zErrors.begin(), zErrors.end()) << endl;
      }

      int diffInliersCount = testNode["diffInliersCount"];
      //cout << "Real objects count = " << (int)testNode["imagesCount"] << endl;
      //cout << "Inliers count = " << diffInliersCount << endl;
      //cout << "FP count = " << (int)testNode["fpCount"] << endl;

      int imagesCount = (int)testNode["imagesCount"];
      int fpCount = (int)testNode["fpCount"];
      double detRate, fpRate;
      if (inliersPosesCount)
      {
        detRate = diffInliersCount / (double)imagesCount;
      }
      else
      {
        detRate = 0.f;
      }

      fpRate = fpCount / (double)imagesCount;

      cout << "Detection rate = " << detRate << endl;
      cout << "FP rate = " <<  fpRate << endl;

      cout << "Count of poses founded several times = " << inliersPosesCount - diffInliersCount << endl;

      testIndex++;
    }
  }

  fs.release();
  return 0;
}

