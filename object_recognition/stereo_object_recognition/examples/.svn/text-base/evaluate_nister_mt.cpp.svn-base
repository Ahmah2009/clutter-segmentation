#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/thread/pthread/mutex.hpp>
#include <stdexcept>
#include <math.h>

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/CvBridge.h>
#include <cv.h>
#include <highgui.h>
#include <image_geometry/pinhole_camera_model.h>
#include <stereo_object_recognition/Coordinate3DProviderDenseStereo.h>
#include "stereo_object_recognition/ObjectDatabase3D.h"
#include "stereo_object_recognition/SynchronizedQueue.h"
#include "stereo_object_recognition/SVDRigidEstimator.h"
#include "stereo_object_recognition/ObjectDetectorNister.h"
#include <boost/timer.hpp>
#include <boost/thread/pthread/condition_variable_fwd.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace cv;
using namespace stereo_object_recognition;
namespace f2d = cv;


const int max_nister_objects = 8;

/**
 * \author Suat Gedikli
 * \date 28. April 2010
 * \brief test class for managing object database, user data and node-callbacks!
 */

class MyObjectRecognizer
{
  protected:
    struct Job
    {
      public:
        stereo_msgs::DisparityImageConstPtr disparity_image;
        Mat left_rect_image;
        int job_id;
    };

  public:
    MyObjectRecognizer(boost::shared_ptr<f2d::FeatureDetector> feature_detector,
                       boost::shared_ptr<f2d::DescriptorExtractor> descriptor_extractor,
                       boost::shared_ptr<f2d::DescriptorMatcher> descriptor_matcher,
                       boost::shared_ptr<ObjectDetector> object_detector = boost::shared_ptr<ObjectDetector>((ObjectDetector*)0) );
    //MyObjectRecognizer( );
    ~MyObjectRecognizer();
    int run();
  protected:
    // Callback methods
    void cameraCallback( const sensor_msgs::CameraInfoConstPtr& camera_info, bool is_left );
    void imageCallback( const sensor_msgs::ImageConstPtr& left_image, const stereo_msgs::DisparityImageConstPtr& disparity_image );

    // helper functions
    bool onKey( char key );
    void printMenu() const;
    void getObjectMask( Mat& maskImage, const Coordinate3DProvider& coordinate_provider, double min_dist, double max_dist ) const;
    void queryCurrentObject() const;
    void loadDatabase();
    void evaluatNisterResults( const vector<ObjectDatabase3D::ObjectMatch3D>& nister_result,
                               const vector<ObjectDatabase3D::ObjectMatch3D>& groundtruth ) const;

    void workerThread();
    void writeFrame( Mat& frame, int job_id );
    
    // member variables
    Mat left_rect_image_; // image to use for feature detection and descriptor extraction
    Mat table_rotation_; // the table pose
    static const double min_point_distance_sqr_; // minimum point distance
    static const double max_point_distance_sqr_; // maximum point distance

    image_geometry::StereoCameraModel  stereo_camera_model_;
    stereo_msgs::DisparityImageConstPtr disparity_image_;
    bool stereo_camera_initialized_;

    // NodeHandles
    ros::NodeHandle node_handle_;
    ros::Subscriber left_camera_sub_;
    ros::Subscriber right_camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> left_rect_image_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage> time_sync_sub_;
    message_filters::Connection time_sync_connection_;

    boost::shared_ptr<f2d::FeatureDetector>     feature_detector_;
    boost::shared_ptr<f2d::DescriptorExtractor> descriptor_extractor_;
    boost::shared_ptr<GeometricValidator3D>     geometric_validator_;
    boost::shared_ptr<ObjectDetector>           object_detector_;

    ObjectDatabase3D object_database_nister_; // Container handling queries etc.
    ObjectDatabase3D object_database_all_; // Container handling queries etc.

    // Following is not handled by ObjectDatabase3D, since application dependend
    vector<Mat> object_views_;   // view of objects in DB
    vector<vector<KeyPoint> > keypoints_; // keypoints of objects in DB
    vector<Vec3d> object_orientations_; // y-Axis of table!

    mutable boost::mutex output_mutex_;
    mutable boost::condition_variable start_condition_;
    SynchronizedQueue<Job> jobs_;
    vector<boost::thread*> worker_threads_;
    map<boost::thread::id, queue<pair<Mat, int> > > frame_queues_;
    queue<Mat> frame_queue_;
};

const double MyObjectRecognizer::min_point_distance_sqr_ = 0.2;
const double MyObjectRecognizer::max_point_distance_sqr_ = 1.1;

MyObjectRecognizer::MyObjectRecognizer(boost::shared_ptr<f2d::FeatureDetector> feature_detector,
                                       boost::shared_ptr<f2d::DescriptorExtractor> descriptor_extractor,
                                       boost::shared_ptr<f2d::DescriptorMatcher> descriptor_matcher,
                                       boost::shared_ptr<ObjectDetector> object_detector )
  : left_rect_image_( )
  , table_rotation_( Mat::eye( 3, 3, CV_64F) )
  , disparity_image_( (stereo_msgs::DisparityImage*)0 )
  , stereo_camera_initialized_( false )
  , time_sync_sub_( left_rect_image_sub_, disparity_sub_, 1000 )
  , feature_detector_( feature_detector )
  , descriptor_extractor_( descriptor_extractor )
  , geometric_validator_( new GeometricValidator3D( pow(0.03,2.0) ) )
  , object_detector_( object_detector )
  , object_database_nister_( descriptor_matcher, geometric_validator_, object_detector_, 0.03 )
  , object_database_all_( descriptor_matcher, geometric_validator_, 0.03 )
{
  boost::function<void(const sensor_msgs::CameraInfoConstPtr&)> right_camera_callback = boost::bind( &MyObjectRecognizer::cameraCallback, this, _1, false );
  boost::function<void(const sensor_msgs::CameraInfoConstPtr&)> left_camera_callback = boost::bind( &MyObjectRecognizer::cameraCallback, this, _1, true );

  left_camera_sub_    = node_handle_.subscribe("/mp_stereo/left/camera_info", 1000, left_camera_callback );
  right_camera_sub_   = node_handle_.subscribe("/mp_stereo/right/camera_info", 1000, right_camera_callback );
  left_rect_image_sub_.subscribe( node_handle_, "/mp_stereo/left/image_rect", 100 );
  disparity_sub_.subscribe( node_handle_, "/mp_stereo/disparity", 100 );
  time_sync_connection_ = time_sync_sub_.registerCallback( &MyObjectRecognizer::imageCallback, this );

  namedWindow("leftRectImage", CV_WINDOW_AUTOSIZE );
  namedWindow("mask image", CV_WINDOW_AUTOSIZE );
  namedWindow("best match", CV_WINDOW_AUTOSIZE );

  loadDatabase();

  // create the workerthreads
  stringstream ss;
  for( unsigned tIdx = 0; tIdx < std::max(unsigned(1),boost::thread::hardware_concurrency()); ++tIdx )
  {
    boost::thread* worker_thread = new boost::thread(&MyObjectRecognizer::workerThread, this);
    worker_threads_.push_back( worker_thread );
    frame_queues_[worker_thread->get_id()].size(); // just creat that element in the map
  }
}

MyObjectRecognizer::~MyObjectRecognizer()
{
  time_sync_connection_.disconnect();

  // remove all existing jobs in queue
  jobs_.clear();
  Job non_valid_job;
  non_valid_job.job_id = -1;

  // add invalid jobs to terminate worker threads
  for( vector<boost::thread*>::iterator it = worker_threads_.begin(); it != worker_threads_.end(); ++it )
    jobs_.push_back( non_valid_job );

  // wait until threads receive invalid jobs and terminate
  for( vector<boost::thread*>::iterator it = worker_threads_.begin(); it != worker_threads_.end(); ++it )
    (*it)->join();

  // so we're done
}

void MyObjectRecognizer::cameraCallback( const sensor_msgs::CameraInfoConstPtr& camera_info, bool is_left )
{
  // temporary objects for creating the stereo camera model
  static image_geometry::PinholeCameraModel left_camera_model;
  static image_geometry::PinholeCameraModel right_camera_model;
  static sensor_msgs::CameraInfoConstPtr left_cam_info( (sensor_msgs::CameraInfo*)0 );
  static sensor_msgs::CameraInfoConstPtr right_cam_info( (sensor_msgs::CameraInfo*)0 );

  if( is_left )
  {
    left_camera_model.fromCameraInfo( camera_info );
    left_cam_info = camera_info;
  }
  else
  {
    //cerr << endl << "got right camera parameters!" << endl;
    right_camera_model.fromCameraInfo( camera_info );
    right_cam_info = camera_info;
  }

  if( left_cam_info.get() != 0 &&  right_cam_info.get() != 0 )
  {
    stereo_camera_model_.fromCameraInfo( left_cam_info, right_cam_info );
    left_camera_sub_.shutdown();
    right_camera_sub_.shutdown();
    stereo_camera_initialized_ = true;
  }
}

void MyObjectRecognizer::imageCallback( const sensor_msgs::ImageConstPtr& image_ptr,
                                        const stereo_msgs::DisparityImageConstPtr& disparity_image_ptr )
{
  sensor_msgs::CvBridge cv_bridge;
  try
  {
    // data to tempImage will be released in destructor of CvBridge !!!
    Mat temp_image = cv_bridge.imgMsgToCv(image_ptr, "mono8");
    temp_image.copyTo( left_rect_image_ );
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error converting ImageConstPtr to IplImage");
  }

  if( stereo_camera_initialized_ )
  {
    static unsigned job_id = 0;
    //queryCurrentObject();
    Job threadContext;
    // smart pointer
    threadContext.disparity_image = disparity_image_ptr;
    // deep copy... since its
    threadContext.left_rect_image = left_rect_image_.clone();
    threadContext.job_id = job_id++;
    jobs_.push_back( threadContext );
  }
  imshow("leftRectImage", left_rect_image_);
}

void MyObjectRecognizer::printMenu() const
{
  cout << endl;
  cout << "======== MENU ========" << endl;
  cout << endl;
  cout << "<Q>  : Quit" << endl;
  cout << "<Esc>: Quit" << endl;
}

int MyObjectRecognizer::run()
{
  int key;
  printMenu();
  do {
      ros::spinOnce();
      if( frame_queue_.size() > 0 )
      {
        imshow("best match", frame_queue_.front() );
        frame_queue_.pop();
      }
      key = waitKey(10);
  } while( key <= 0 || onKey( (char) (key & 0xFF) ) );
    
  return 0;
}

void MyObjectRecognizer::evaluatNisterResults( const vector<ObjectDatabase3D::ObjectMatch3D>& nister_result,
                                               const vector<ObjectDatabase3D::ObjectMatch3D>& groundtruth ) const
{
  // This call is not thread-safe -> has to be mutexed from outside!
  static unsigned best_nister_match[7] = { 0, 0, 0, 0, 0, 0, 0 };

  if( nister_result.size() > 0 && groundtruth.size() > 0 )
    {
      // check the first 8 from brute force
      unsigned mIdx;
      for( mIdx = 0; mIdx < (unsigned)std::min(4,(int)groundtruth.size()); ++mIdx )
      {
        if( groundtruth[mIdx].object_ID == nister_result[0].object_ID )
        {
          best_nister_match[mIdx]++;
          break;
        }
      }

      if( mIdx == 4 ) //not within first 8
      {
        best_nister_match[4]++;
      }
    }
    else if( groundtruth.size() > 0 )
    { // no match from nister
      best_nister_match[5]++;
    }
    else
    {
      best_nister_match[6]++;
    }

    unsigned total = 0;
    for( unsigned mIdx = 0; mIdx < 7; ++mIdx )
      total += best_nister_match[mIdx];

    cout << "best nister match result: " << endl;
    for( unsigned mIdx = 0; mIdx < 7; ++mIdx )
    {
      cout << mIdx << ".\t" << best_nister_match[mIdx] << " = " << (float)best_nister_match[mIdx] * 100 / (float)total << "%";
      if( mIdx == 4 )
        cout << " ( > 4 )" << endl;
      else if( mIdx == 5 )
        cout << " ( no match from nister )" << endl;
      else if( mIdx == 6 )
        cout << " ( no match at all )" << endl;
      else
        cout << endl;
    }
}

void MyObjectRecognizer::getObjectMask( Mat& mask_image, const Coordinate3DProvider& coordinate_provider, double min_point_distance_sqr, double max_point_distance_sqr ) const
{
  // remove all pixels out of given range [minDist;maxDist]

  mask_image.setTo( Scalar( 0, 0, 0, 0) );

  cv::Point3d coordinate;
  for( int yIdx = 0; yIdx < mask_image.rows; ++yIdx )
  {
    for( int xIdx = 0; xIdx < mask_image.cols; ++xIdx )
    {
      if( coordinate_provider.getCoordinate( cv::Point2d(xIdx,yIdx), coordinate ) )
      {
        double distanceSQR = coordinate.x * coordinate.x + coordinate.y * coordinate.y + coordinate.z * coordinate.z;
        if( distanceSQR >= min_point_distance_sqr && distanceSQR <= max_point_distance_sqr )
        {
          mask_image.at<unsigned char>( yIdx, xIdx ) = 255;
        }
      }
    }
  }

  cv::Mat morph;

  // close holes
  morphologyEx( mask_image, morph, MORPH_CLOSE, Mat::ones( 17, 17, CV_8U), cv::Point(-1, -1) );

  // remove small blobs
  morphologyEx( morph, morph, MORPH_OPEN, Mat::ones( 17, 17, CV_8U), cv::Point(-1, -1) );

  // remove edges
  erode( morph, morph, Mat::ones( 11, 11, CV_8U), cv::Point(-1, -1) );

  bitwise_and( morph, mask_image, mask_image );
}

bool MyObjectRecognizer::onKey( char key )
{
  switch( toupper(key) )
  {
    case 'Q':
    case  27:
        return false;
        break;
  }
  return true;
}

void  MyObjectRecognizer::loadDatabase()
{
  ifstream file( "objects.db", ios_base::in );
    if( file.fail() )
      throw( std::runtime_error("could not open file objects.db"));
    file >> object_database_all_;
    file.seekg( ios_base::beg );
    file >> object_database_nister_;
  file.close();

  unsigned size = object_database_all_.objectsCount();
  cout << "loaded object DB with " << size << " objects" << endl;

  file.open("objectKeypoints.db", ios_base::in );
  if( file.fail() )
    throw( std::runtime_error("could not open file objectKeypoints.db"));
  keypoints_.resize( size );
  unsigned objIdx = 1;
  for( vector<vector<KeyPoint> >::iterator it= keypoints_.begin(); it != keypoints_.end(); ++it, ++objIdx )
  {
    uint32_t kSize;
    file >> kSize;
    cout << "reading " << kSize << " keypoints for object " << objIdx<< endl;
    it->resize( kSize );
    for( vector<KeyPoint>::iterator kIt = it->begin(); kIt != it->end(); ++kIt )
      file >> kIt->angle >> kIt->class_id >> kIt->octave >> kIt->pt.x >> kIt->pt.y >> kIt->response >> kIt->size;
  }
  file.close();

  object_views_.resize( size );
  char view_file_name[1024];
  for( unsigned  viewIdx = 0; viewIdx < size; ++viewIdx )
  {
    sprintf( view_file_name, "view%03d.png", viewIdx );
    object_views_[viewIdx] = imread( view_file_name, -1 );
  }
}

void MyObjectRecognizer::workerThread()
{
  stringstream ss;
  while(true)
  {
    Job threadContext = jobs_.pop_front();

    // if not valid jobid => stop workerThread!
    if( threadContext.job_id < 0 )
    {
      cout << "got exit request => leaving thread function " << boost::this_thread::get_id() << endl;
      break;
    }
    cout << boost::this_thread::get_id() << " :: got job: " << threadContext.job_id << " jobs left: " << jobs_.size() << endl;

    stereo_msgs::DisparityImageConstPtr disparity_image = threadContext.disparity_image;
    const Mat& left_rect_image = threadContext.left_rect_image;

    Mat mask_image( Size(disparity_image->image.width, disparity_image->image.height), CV_8UC1 );

    Coordinate3DProviderDenseStereo coordinate_provider( *disparity_image, stereo_camera_model_ );
    getObjectMask( mask_image, coordinate_provider, min_point_distance_sqr_, max_point_distance_sqr_ );

    vector<KeyPoint> keypoints;
    feature_detector_->detect( left_rect_image, keypoints, mask_image );

    Mat descriptors;
    descriptor_extractor_->compute( left_rect_image, keypoints, descriptors );
    vector<Point3d> position_data;
    position_data.reserve( keypoints.size() );
    Point3d coordinate;
    for( unsigned kIdx = 0; kIdx < keypoints.size();  )
    {
      if (coordinate_provider.getCoordinate( keypoints[kIdx].pt, coordinate ) )
      {
        position_data.push_back( coordinate );
        ++kIdx;
      }
      else
      {
        // remove keypoint and descriptor => copy last row / keypoint to current row / keypoint and remove last row / keypoint!
        // problem extracting descriptors after removing keypoints: extractor my throw away keypoints!!!!
        keypoints[kIdx] = keypoints.back();
        keypoints.pop_back();

        Mat currentRow = descriptors.row( kIdx );
        descriptors.row( descriptors.rows-1 ).copyTo( currentRow );
        descriptors = descriptors.rowRange(0, descriptors.rows-1 );
      }
    } // for
    boost::timer timer;
    vector<ObjectDatabase3D::ObjectMatch3D> object_matches_nister = object_database_nister_.queryObject( descriptors, position_data );
    cout << "nister query time: " << timer.elapsed() << endl;
    timer.restart();
    vector<ObjectDatabase3D::ObjectMatch3D> object_matches_all = object_database_all_.queryObject( descriptors, position_data );
    cout << "brute force query time: " << timer.elapsed() << endl;


    output_mutex_.lock();
    evaluatNisterResults( object_matches_nister, object_matches_all );
    output_mutex_.unlock();

    Mat match_view(left_rect_image.rows, left_rect_image.cols * 2, CV_8UC3 );
    Mat match_view_scaled(left_rect_image.rows >> 1, left_rect_image.cols, CV_8UC3 );

    if( object_matches_all.size() > 0 )
    {
      cout << endl << " best matches from brute force" << endl;
      cout << endl << "#\tid\tscore\tinliers / correspondences" << endl;
      for( unsigned mIdx = 0; mIdx < (unsigned)std::min(max_nister_objects,(int)object_matches_all.size()); ++mIdx )
      {
        cout << mIdx << "\t" << object_matches_all[mIdx].object_ID << "\t" << object_matches_all[mIdx].score << "\t" << object_matches_all[mIdx].geometric_match_indices.size() << " / " << object_matches_all[mIdx].descriptor_match_indices.size() << endl;
      }
      cout << " vs. best matches with nister" << endl;
      cout << endl << "#\tid\tscore\tinliers / correspondences" << endl;
      for( unsigned mIdx = 0; mIdx < (unsigned)std::min(max_nister_objects,(int)object_matches_nister.size()); ++mIdx )
      {
        cout << mIdx << "\t" << object_matches_nister[mIdx].object_ID << "\t" << object_matches_nister[mIdx].score << "\t" << object_matches_nister[mIdx].geometric_match_indices.size() << " / " << object_matches_nister[mIdx].descriptor_match_indices.size() << endl;
      }

      ObjectDatabase3D::ObjectMatch3D* best_match_ptr;

      if( object_matches_nister.size() > 0 )
      {
        cout << "displaying best match from nister query" << endl;
        best_match_ptr = &object_matches_nister[0];
      }
      else
      {
        cout << "nister returned no result => displaying best matching view from brute force matcher" << endl;
        best_match_ptr = &object_matches_all[0];
      }

      ObjectDatabase3D::ObjectMatch3D& best_match = *best_match_ptr;
      unsigned bestOIdx = best_match.object_ID;

      btTransform object_transform;
      btMatrix3x3 rotation(
        best_match.rotation_matrix.at<double>(0,0), best_match.rotation_matrix.at<double>(0,1), best_match.rotation_matrix.at<double>(0,2),
        best_match.rotation_matrix.at<double>(1,0), best_match.rotation_matrix.at<double>(1,1), best_match.rotation_matrix.at<double>(1,2),
        best_match.rotation_matrix.at<double>(2,0), best_match.rotation_matrix.at<double>(2,1), best_match.rotation_matrix.at<double>(2,2)
      );

      btQuaternion qRot;
      rotation.getRotation( qRot );


      object_transform.setOrigin( btVector3( best_match.translation_vector[0], best_match.translation_vector[1], best_match.translation_vector[2] ) );
      object_transform.setRotation( qRot );

      btTransform table_transform = object_transform;
      btMatrix3x3 table_rotation( table_transform.getRotation() );

      Mat left_image_part( match_view, Rect( 0, 0, left_rect_image.cols, left_rect_image.rows ) );
      Mat right_image_part( match_view, Rect( left_rect_image.cols, 0, left_rect_image.cols, left_rect_image.rows ) );

      IplImage left_image_part_ipl =  (IplImage)left_image_part;
      IplImage right_image_part_ipl =  (IplImage)right_image_part;

      IplImage left_rect_image_ipl = (IplImage)left_rect_image;
      IplImage object_view_ipl = (IplImage)object_views_[bestOIdx];

      cvCvtColor( &left_rect_image_ipl, &left_image_part_ipl, CV_GRAY2BGR );
      cvCvtColor( &object_view_ipl, &right_image_part_ipl, CV_GRAY2BGR );

      // Draw matching descriptors
      for( vector<pair<unsigned,unsigned> >::const_iterator it = best_match.descriptor_match_indices.begin(); it != best_match.descriptor_match_indices.end(); ++it )
      {
        Point from = keypoints[it->second].pt;
        Point to   = keypoints_[bestOIdx][it->first].pt;
        to.x += left_rect_image.cols;
        line( match_view, from, to, Scalar( 0, 0, 255, 0 ), 1 );
      }

      // Draw geometric correct correspondences!!!
      for( vector<unsigned>::const_iterator it = best_match.geometric_match_indices.begin(); it != best_match.geometric_match_indices.end(); ++it )
      {
        Point from = keypoints[best_match.descriptor_match_indices[*it].second].pt;
        Point to   = keypoints_[bestOIdx][best_match.descriptor_match_indices[*it].first].pt;
        to.x += left_rect_image_.cols;

        int octave = max(keypoints[best_match.descriptor_match_indices[*it].second].octave, keypoints_[bestOIdx][best_match.descriptor_match_indices[*it].first].octave);
        Scalar color( 128 + 32 * octave, 0, 0, 0 );
        line( match_view, from, to, color, 1 + octave );
      }

      //unsigned maxPossibleCorrs = std::min( keypoints.size(), _objectDatabase.getObjectFeatures( bestMatch.objectID ).second.size() );
      char row_string[1024];
      table_transform = object_transform;
      table_rotation = btMatrix3x3( table_transform.getRotation() );

      Scalar textColor( 0, 255, 255, 0 );

      ss.str("");
      ss << "Thread : " << boost::this_thread::get_id() <<" job: " << threadContext.job_id;
      putText( match_view, ss.str().c_str(), Point( 10, 20 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "keypoints in query image %4d", (int)keypoints.size() );
      putText( match_view, row_string, Point( 10, 60 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "keypoints in db image %4d", (int)object_database_nister_.getObjectFeatures( best_match.object_ID ).second.size() );
      putText( match_view, row_string, Point( 10+left_rect_image.cols, 60 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "Transformation between DB view and current view" );
      putText( match_view, row_string, Point( 10, 100 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "% 1.2f % 1.2f % 1.2f % 1.2f", table_rotation[0].x(),  table_rotation[0].y(), table_rotation[0].z(), table_transform.getOrigin().x() );
      putText( match_view, row_string, Point( 10, 140 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "% 1.2f % 1.2f % 1.2f % 1.2f", table_rotation[1].x(),  table_rotation[1].y(), table_rotation[1].z(), table_transform.getOrigin().y() );
      putText( match_view, row_string, Point( 10, 180 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "% 1.2f % 1.2f % 1.2f % 1.2f", table_rotation[2].x(),  table_rotation[2].y(), table_rotation[2].z(), table_transform.getOrigin().z() );
      putText( match_view, row_string, Point( 10, 220 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "rotation around axis: % 1.2f % 1.2f % 1.2f with angle: % 1.2f", qRot.getAxis().x(),  qRot.getAxis().y(), table_rotation[2].z(), qRot.getAngle() );
      putText( match_view, row_string, Point( 10, 260 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "% 4d feature correspondences", (int)best_match.descriptor_match_indices.size() );
      putText( match_view, row_string, Point( 10, 340 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

      sprintf( row_string, "% 4d inliers from geometric check = % 1.2f %%", (int)best_match.geometric_match_indices.size(), best_match.geometric_match_indices.size() * 100.0 / best_match.descriptor_match_indices.size() );
      putText( match_view, row_string, Point( 10, 380 ), FONT_HERSHEY_PLAIN, 2.0, textColor, 2  );

    }
    else
    {
      Scalar textColor( 0, 255, 255, 0 );
      match_view.setTo( Scalar( 0, 0, 0, 0) );
      Mat left_image_part( match_view, Rect( 0, 0, left_rect_image.cols, left_rect_image.rows ) );
      IplImage left_image_part_ipl =  (IplImage)left_image_part;
      IplImage left_rect_image_ipl = (IplImage)left_rect_image;
      cvCvtColor( &left_rect_image_ipl, &left_image_part_ipl, CV_GRAY2BGR );

      ss.str("");
      ss << "Thread : " << boost::this_thread::get_id() <<" job: " << threadContext.job_id << " :: no match found!";
      putText( match_view, ss.str().c_str(), Point( 10, 80 ), FONT_HERSHEY_PLAIN, 4.0, textColor, 2  );
    }

    resize( match_view, match_view_scaled, match_view_scaled.size() );
    writeFrame( match_view_scaled, threadContext.job_id );
  }
}

void MyObjectRecognizer::writeFrame( Mat& frame, int job_id )
{
  static boost::mutex video_mutex;

  // lock the mutex
  mutex::scoped_lock lock( video_mutex );
  frame_queues_[ boost::this_thread::get_id() ].push( pair<Mat, int>( frame, job_id ) );

  // now check if all threads returned at least one result!

  boost::thread::id thread_id = boost::this_thread::get_id();
  for( map<boost::thread::id, queue<pair<Mat, int> > >::iterator it = frame_queues_.begin(); it != frame_queues_.end(); ++it )
  {
    // at least one thread doe not return a frame => nothing to do until we get one frame
    if( it->second.size() == 0 )
      return;

    if( frame_queues_[thread_id].front().second > frame_queues_[it->first].front().second )
      thread_id = it->first;
  }

  // We got at least one frame per thread
  // thread_id contains the thread with the lowest frame-number (job_id) =>  write the frame

  pair<Mat, int> next_frame = frame_queues_[thread_id].front();
  frame_queues_[thread_id].pop();
  frame_queue_.push( next_frame.first );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  //boost::shared_ptr<f2d::FeatureDetector> featureDetector( new f2d::SurfFeatureDetector );
  //boost::shared_ptr<f2d::FeatureDetector> featureDetector( new f2d::PyramidAdapter( new f2d::HarrisFeatureDetector(100), 3 ) );

  //boost::shared_ptr<f2d::FeatureDetector> featureDetector( new f2d::PyramidAdapter( cv::Ptr<f2d::FeatureDetector>(new f2d::FastFeatureDetector(5) ), 3 ) );
  boost::shared_ptr<f2d::FeatureDetector> featureDetector( new f2d::FastFeatureDetector(3,true) );
  boost::shared_ptr<f2d::DescriptorExtractor> descriptorExtractor( new f2d::CalonderDescriptorExtractor<float>("calonder.rtc") );
  //boost::shared_ptr<f2d::DescriptorExtractor> descriptorExtractor( new f2d::SurfDescriptorExtractor( ) );
  boost::shared_ptr<f2d::DescriptorMatcher> descriptorMatcher( new f2d::BruteForceMatcher<f2d::L2<float> > );
  boost::shared_ptr<ObjectDetector> object_detector( new ObjectDetectorNister<float>("holidays.tree","holidays.weights",max_nister_objects) );
  //boost::shared_ptr<ObjectDetector> object_detector( (ObjectDetector*)0 );
  MyObjectRecognizer objectRecognizer( featureDetector, descriptorExtractor, descriptorMatcher, object_detector );
  return objectRecognizer.run();
}

