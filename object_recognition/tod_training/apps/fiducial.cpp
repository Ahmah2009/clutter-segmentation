/*
 * fiducial.cpp
 *
 *  Created on: 2011
 *      Author: Ethan Rublee
 */

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <tod/training/pose.h>
#include <tod/training/file_io.h>
#include <boost/format.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace tod;
using namespace std;
using namespace cv;

namespace po = boost::program_options;

namespace
{

struct fiducial_options
{
  std::string output;
  float square_size;
  int w,h;
  float x,y,z;
  Fiducial::Type type;
};

int options(int ac, char ** av, fiducial_options& opts)
{
  // Declare the supported options.
  po::options_description desc("fiducial options");
  desc.add_options()("help", "produce help message");
  desc.add_options()("output,o", po::value<std::string>(&opts.output)->default_value("fiducial.yml"),
      "Output fiducial yaml file name.");
  desc.add_options()("square_size,s", po::value<float>(&opts.square_size)->default_value(0.02),
       "square size, in meters.");
  desc.add_options()("board_width,w", po::value<int>(&opts.w)->default_value(8),
        "Number of inside corners width wise");
  desc.add_options()("board_height,h", po::value<int>(&opts.h)->default_value(6),
          "Number of inside corners height wise");
  desc.add_options()("offset_x,x", po::value<float>(&opts.x)->default_value(0),
            "offset of topleft corner from origin, x axis");
  desc.add_options()("offset_y,y", po::value<float>(&opts.y)->default_value(0),
              "offset");
  desc.add_options()("offset_z,z", po::value<float>(&opts.z)->default_value(0),
              "offset");
  desc.add_options()("type,T", po::value<int>((int*)(&opts.type))->default_value(int(Fiducial::CHECKER_BOARD)),
               (boost::format("pattern type,%d -- CHECKER_BOARD, %d -- DOTS") % int(Fiducial::CHECKER_BOARD) % int(Fiducial::DOTS)).str().c_str());
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  return 0;
}

}

int main(int argc, char *argv[])
{
  fiducial_options opts;
  if (options(argc, argv, opts))
    return 1;

  vector<Size> corner_counts(1);
  corner_counts[0] = Size(opts.w,opts.h); //big board first -- for ambiguities in size TODO better fiducial markers.

  vector<float> spacings(1, opts.square_size /*2 cm*/);

  vector<Point3f> offsets(1);
  offsets[0] = Point3f(opts.x, opts.y, opts.z);

  Fiducial f(corner_counts, spacings, offsets,opts.type);
  FileStorage fs(opts.output, FileStorage::WRITE);
  fs << "fiducial";
  f.write(fs);
  fs.release();

  return 0;
}
