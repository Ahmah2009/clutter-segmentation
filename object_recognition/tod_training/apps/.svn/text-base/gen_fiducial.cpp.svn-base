#include <opencv2/opencv.hpp>

#include <tod/training/pose.h>
#include <tod/training/file_io.h>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{ float scale =1;
  if(argc == 2)
   scale = atof(argv[1]);
  vector<Size> corner_counts(2);
  corner_counts[0] = Size(3, 8); //big board first -- for ambiguities in size TODO better fiducial markers.
  corner_counts[1] = Size(3, 6);

  vector<float> spacings(2, 0.02 *scale /*2 cm*/);

  vector<Point3f> offsets(2);
  offsets[0] = Point3f(-.070, 0.070, 0) * scale;
  offsets[1] = Point3f(-0.050, -0.110, 0) * scale; // these are measured from art/board.02.svg



  Fiducial f(corner_counts, spacings, offsets);
  FileStorage fs("fiducial.yml", FileStorage::WRITE);
  fs << "fiducial";
  f.write(fs);
  fs.release();

  cout << "See newly generated fiducial.yml" << endl;

  return 0;
}
