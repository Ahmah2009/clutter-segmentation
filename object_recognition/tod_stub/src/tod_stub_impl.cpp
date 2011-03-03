//my header
// THIS IS WHERE YOU CAN IMPLEMENT YOU CODE WHICH STUBS OUT "MyTrainer and MyDetector" but you can name these anything and link it to 
// a separate file.
#include <tod_stub/tod_stub_impl.h>

//pcl library code
#include <pcl/registration/transforms.h>
#include <pcl/filters/passthrough.h>

//pcl visualization
#include <pcl_visualization/cloud_viewer.h>

//opencv gui stufff
#include <opencv2/highgui/highgui.hpp>

//for converting to from eigen from opencv mats
//namespace Eigen = Eigen3;
#include <opencv2/core/eigen.hpp>

#include <boost/foreach.hpp>
#include <utility>

namespace tod_stub
{
namespace
{
struct MyData
{
  std::string object_id;
  cv::Mat R;
  cv::Mat T;
};
class MyTrainer : public Trainer
{
public:
  MyTrainer(const std::string& parameter_file) :
    parameter_file(parameter_file), cloud_(new Cloud_t()), cloud_viewer_("train 3d data")
  {
  }

  virtual void process(const TrainData& data) throw (tod_exception)
  {
    //pose_drawer_.setRT(data.R, data.T, "object_frame");

    imshow("train RGB data", data.frame.image);

    //check for user quick suggestions.
    char key = cv::waitKey(20);
    if (cloud_viewer_.wasStopped() || key == 'q')
      throw QUIT; //this will be caught and program properly exited

    std::cout << "training..." << std::endl;

    //get rid of nans in the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(data.frame.cloud, *cloud_, indices);

    //convert the tranform from our fiducial markers to
    //the Eigen
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Vector3f T;
    cv::cv2eigen(data.T, T);
    cv::cv2eigen(data.R, R);

    //get the inverse transform to bring the point cloud's into the
    //same coordinate frame
    Eigen::Affine3f transform;
    transform = Eigen::AngleAxisf(R.transpose());
    transform *= Eigen::Translation3f(-T);
    //transform the cloud in place
    pcl::transformPointCloud(*cloud_, *cloud_, transform);

    //use the pass through filters to segment the object
    pass_z.setInputCloud(cloud_);
    pass_z.filter(*cloud_);
    pass_y.setInputCloud(cloud_);
    pass_y.filter(*cloud_);
    pass_x.setInputCloud(cloud_);
    pass_x.filter(*cloud_);

    //show the cloud in the gui
    cloud_viewer_.showCloud(*cloud_);

    //************************
    //do awesome training here.
    //************************

    //Persist this observation
    MyData m;
    m.object_id = data.object_id;
    m.R = data.R.clone();
    m.T = data.T.clone();
    my_data[data.object_id].push_back(m);

    //possibly write to disk here.


  }

  virtual void onInit()
  {
    cv::namedWindow("train RGB data", CV_WINDOW_KEEPRATIO);
    //add a reference to the cloud_viewer, so that it doesn't copy our pose_drawer_
    cloud_viewer_.runOnVisualizationThread(boost::ref(pose_drawer_));
    //************************
    //do any other initialization here
    //************************
    //SAMPLE USER INIT
    output_dir = ".";
    bool do_segment = true;
    float box = 0.25; // size in meters of the segmentation box.
    cv::FileStorage fs(parameter_file, cv::FileStorage::READ);
    if (fs.isOpened())
    {
      box = (float)fs["box_size"];
      do_segment = (int)fs["do_segmentation"];
      output_dir = (std::string)fs["output_directory"];
    }
    else
    {
      cv::FileStorage sample_fs(parameter_file, cv::FileStorage::WRITE);
      if (sample_fs.isOpened())
      {
        sample_fs << "box_size" << box;
        sample_fs << "do_segment" << (int)do_segment;
        sample_fs << "output_directory" << output_dir;
      }
      std::cerr << "could not open the parameter file! sample made." << std::endl;
    }
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.01, box);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-box, box);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-box, box);
  }
  virtual void onFinish()
  {
    //************************
    //do any clean up here, or serialization
    //************************

    typedef std::map<std::string, std::list<MyData> >::value_type dp_t;
    //serialize all my data
    BOOST_FOREACH(const dp_t& dp, my_data )
    {
      const std::list<MyData>& data = dp.second;
      if (data.empty())
        continue;
      cv::FileStorage fs(output_dir + "/" + dp.first + ".yml", cv::FileStorage::WRITE);
      fs << data.front().object_id << "[";
      BOOST_FOREACH(const MyData& m, data)
      {
        fs << "{" << "object_id" << m.object_id << "R" << m.R << "T" << m.T << "}";
      }
      fs << "]";
    }
  }
  ~MyTrainer()
  {

  }
private:
  std::string parameter_file; //!< use this to load up data from
  std::string output_dir; //!< where will i save my data to?
  Cloud_t::Ptr cloud_;
  pcl_visualization::CloudViewer cloud_viewer_;
  PoseDrawer3d pose_drawer_; // use this to visualize the fiducial poses.
  pcl::PassThrough<Cloud_t::PointType> pass_z, pass_x, pass_y;
  std::map<std::string, std::list<MyData> > my_data;

};

class MyDetector : public Detector
{
public:
  MyDetector(const std::string& parameter_file)
  {
  }
  virtual void detect(const FrameData& data, std::vector<Result>& results) throw (tod_exception)
  {
    //Below just stubs out some random detection -- first some random results that contain
    // random Rotation matrix, Random translation, random object ID and random confidence
    std::cout << "detecting..." << std::endl;
    results.clear();
    results.reserve(10);
    for (int i = 0; i < 10; i++)
    {
      Result r;
      r.R = cv::Mat_<double>::eye(3, 3);
      r.T = (cv::Mat_<double>(3, 1) << (std::rand() % 1000) / 2000.0, (std::rand() % 1000) / 2000.0, (std::rand()
          % 1000) / 2000.0);
      r.object_id = (boost::format("toto_%04d") % (std::rand() % 30)).str();
      r.confidence = (std::rand() % 1000) / 1000.0;
      results.push_back(r);
    }
  }
  virtual void onInit()
  {

  }
  virtual void onFinish()
  {

  }
private:
};
}

Detector* createDetector(const std::string& parameter_file)
{
  return new MyDetector(parameter_file);
}
Trainer* createTrainer(const std::string& parameter_file)
{
  return new MyTrainer(parameter_file);
}
}
