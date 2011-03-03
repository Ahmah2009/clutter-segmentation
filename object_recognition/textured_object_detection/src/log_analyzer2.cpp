#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <iomanip>
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

  int fileIndex = 1;
  while(true)
  {
    FileNode fileNode;
    stringstream out;
    out << fileIndex;
    string fileNodeName = "file" + out.str();
    fileNode = fs[fileNodeName];

    if (fileNode.empty())
    {
      break;
    }

    int testIndex = 1;
    vector<string> names;
    vector<double> fpRates;
    vector<double> detRates;

    string testFolder = fileNode["bagFileName"];
    cout << endl << testFolder << endl;
    int fpCount = (int)fileNode["fpCount"];
    cout << "FP: " << setiosflags(ios::left) << fpCount << endl;

    while (true)
    {
      FileNode testNode;
      stringstream out;
      out << testIndex;
      string testNodeName = "test" + out.str();
      testNode = fileNode[testNodeName];
      if (testNode.empty())
      {
        break;
      }
      else
      {
        string testFolder = "";
        testNode["testFolder"] >> testFolder;
        names.push_back(testFolder);

        int inliersPosesCount = 0;
        testNode["allInliersCount"] >> inliersPosesCount;

        int diffInliersCount = testNode["diffInliersCount"];
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

        detRates.push_back(detRate);
        fpRates.push_back(fpRate);
        testIndex++;
      }
    }

    cout << endl;
    for (size_t ind = 0; ind < names.size(); ind++)
    {
      cout << setw(60) << names[ind] << setw(10) << detRates[ind] << setw(10) << fpRates[ind] << endl;
    }

    fileIndex++;
  }

  fs.release();
  return 0;
}

