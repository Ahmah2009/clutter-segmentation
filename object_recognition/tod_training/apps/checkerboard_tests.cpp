/*
 * checkerboard_pe.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: erublee
 */

/*
 * masker.cpp
 *
 *  Created on: Nov 16, 2010
 *      Author: erublee
 */

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

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

namespace po = boost::program_options;

struct pe_options
{
  std::string camera_file;
  Size board_size;
  float square_size;
  int n_threads;
  std::string image_list;
  std::string directory;
};

int options(int ac, char ** av, pe_options& opts)
{

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("width,w", po::value<int >(&opts.board_size.width)->default_value(5),
                     "Number of inside corners, width wise, columns. Default is 5.");
  desc.add_options()("height,h", po::value<int >(&opts.board_size.height)->default_value(4),
                     "Number of inside corners, height wise, rows. Default is 4.");
  desc.add_options()("images,I", po::value<string>(&opts.image_list),
                     "The list of images, use ls *.png > images.txt. Required.");
  desc.add_options()("threads,j", po::value<int>(&opts.n_threads)->default_value(1),
                     "The number of threads to use during pos estimation, think jobs.");
  desc.add_options()("directory,d", po::value<string>(&opts.directory)->default_value("./"),
                     "The directory that the images are in. Also used for storing the pose estimations.");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  //  if (!vm.count("camera")) {
  //      cout << "Must supply a camera calibration file" << "\n";
  //      cout << desc << endl;
  //      return 1;
  //  }

  if (!vm.count("images"))
  {
    cout << "Must supply an image list file. Use something like ls *.png > images.txt" << "\n";
    cout << desc << endl;
    return 1;
  }

  return 0;

}

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

#define filename(arg) (directory+"/"+arg)

#define mout(x) {boost::mutex::scoped_lock sl(cout_lock);cout << x;}

struct pose_worker
{
  pose_worker(const std::string& directory, const list<string>& images, const ImageBasedPE* posest) :
    directory(directory), images(images), posest(posest)
  {

  }
  void operator()()
  {

    vector<Size> corner_counts(2);
    corner_counts[1] = Size(4,5);
    corner_counts[0] = Size(3,6);

    vector<float> spacings(2, 0.02 /*2 cm*/);

    vector<Point3f> offsets(2);
    offsets[1] = Point3f(-0.050,-0.110, 0); // these are measured from art/board.02.svg
    offsets[0] = Point3f(-.070, 0.070, 0);

    Fiducial f(corner_counts, spacings, offsets);
    FileStorage fs("fiducial.yml", FileStorage::WRITE);
    fs << "fiducial";
    f.write(fs);
    fs.release();
    foreach( const string& x, images )
    {
      cv::Mat image = imread(filename(x));
      cv::Mat gray;
      Camera c;
          c.image_size = image.size();
          c.K = cv::Mat::ones(3,3,CV_32F);
          c.K.at<float>(0,0) = 500;
          c.K.at<float>(1,1) = 500;
          c.K.at<float>(0,2) = c.image_size.width/2;
          c.K.at<float>(1,2) = c.image_size.height/2;
      cvtColor(image, gray, CV_RGB2GRAY);
      mout("finding checkerboards" << endl);
      vector<vector<Point2f> > corners;
      vector<bool> found;
      f.detect(gray, corners, found);
      f.draw(image, corners, found);
      FiducialPoseEstimator fe(f,c);
      PoseRT pose = fe.estimatePose(gray);
      if(pose.estimated){
      PoseDrawer(image,c.K,pose);
      }else
        image = ~image;
      imshow("patterns found", image);
      waitKey();
    }

  }

  static boost::mutex cout_lock;
  static boost::mutex outfile_lock;
  string directory;

  list<string> images;
  const ImageBasedPE* posest;
};

boost::mutex pose_worker::cout_lock;
boost::mutex pose_worker::outfile_lock;

int main(int argc, char *argv[])
{
  pe_options opts;
  if (options(argc, argv, opts))
    return 1;

  //read in the image list provided on the command line
  ifstream fin((opts.directory + "/" + opts.image_list).c_str());
  list<string> images = getImageList(fin);
  if (images.empty())
  {
    cerr << "could not read images list.  Try creating the list with:\nls *.png > images.txt" << endl;
    return 1;
  }

  // Camera camera(opts.camera_file, Camera::OPENCV_YAML);
  // CheckerboardPoseEstimator pose_est(opts.board_size, opts.square_size, camera);

  //spawn some threads so that we get this done in a reasonable amount of time

  //first split the image list into the number of threads that we want
  std::vector<std::list<std::string> > vlist = splitList(images, opts.n_threads);

  //create a thread for each sublist
  boost::thread_group threads;
  foreach(const std::list<std::string>& x,vlist)
        {
          threads.create_thread(pose_worker(opts.directory, x, 0));
        }

  //join all the threads that we spawned
  threads.join_all();

  return 0;
}
