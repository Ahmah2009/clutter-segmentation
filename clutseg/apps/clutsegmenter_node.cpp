/**
 * A ROS node that estimates the pose of one rigid textured object.
 * This ROS node subscribes to 
 *      1) an input point cloud
 *      2) a corresponding camera image
 * and subsequently publishes 
 *      1) the inliers point cloud belonging to the rigid textured object. 
 *      2) the name of the recognized object
 *      3) the estimated pose (6dof) of the recognized object
 *
 * See also:
 *      object_recognition/tod_detecting/apps/recognition_node.cpp
 */

#include "clutseg/clutseg.h"
#include "clutseg/result.h"
#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv_candidate/Camera.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/mutex.hpp>

//msg synchronisation
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <clutseg/ClutsegObject.h>

using namespace std;
using namespace message_filters;
typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
							 sensor_msgs::Image > MySyncPolicy;

class ClutterSegmenter
{
protected:
  ros::NodeHandle n;

public:
  ros::Publisher inliers_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher object_publisher;
  image_transport::Publisher hud_publisher;
  std::string input_cloud_topic, input_image_topic, modelbase, camera_info; 
  message_filters::Subscriber<sensor_msgs::Image> camera_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> synchronizer_;
  message_filters::Connection sync_connection_;
  clutseg::PointCloudT query_cloud;
  cv::Mat query_image;
  cv::Mat query_image_bgr8;
  sensor_msgs::CvBridge bridge;
  clutseg::Clutsegmenter sgm;
  ros::ServiceServer service;
  sensor_msgs::PointCloud2 inliers_msg;
  boost::mutex lock;
  ////////////////////////////////////////////////////////////////////////////////
  ClutterSegmenter (ros::NodeHandle &n) : n(n),  synchronizer_( MySyncPolicy(1), cloud_sub_, camera_sub_)
  {
    inliers_publisher = n.advertise<sensor_msgs::PointCloud2>("clutseg_inliers", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("clutseg_pose", 1);
    object_publisher = n.advertise<std_msgs::String>("clutseg_object", 1);

    // This one is just for debugging purposes. It publishes a kind of heads-up display 
    // showing pose and inliers.
    image_transport::ImageTransport img_transp(n);
    hud_publisher = img_transp.advertise("clutseg_hud", 1);

    n.param("input_cloud_topic", input_cloud_topic, std::string("/kinect_head/camera/rgb/points"));
    n.param("input_image_topic", input_image_topic, std::string("/kinect_head/camera/rgb/image_color"));  
    n.param("modelbase", modelbase, std::string("./data/orb.tar"));
    n.param("camera_info", camera_info, std::string("./data/camera.yml"));

    // // synchronized subscriptions 
    camera_sub_.subscribe( n, input_image_topic, 1000 );
    cloud_sub_.subscribe( n, input_cloud_topic, 1000 );
    sync_connection_ = synchronizer_.registerCallback( &ClutterSegmenter::callback, this );

    sgm = clutseg::Clutsegmenter(modelbase, true);
    cv::namedWindow("hud");

    service = n.advertiseService("clutseg_inliers", &ClutterSegmenter::inliersCallback, this);
    ROS_INFO("[ClutterSegmenter:] service <clutseg_inliers> is up");

    ROS_INFO ("[ClutterSegmenter:] Constructor up");
  }

  bool inliersCallback(clutseg::ClutsegObject::Request  &req,
		       clutseg::ClutsegObject::Response &res )
  {
    boost::mutex::scoped_lock mutex(lock);
    res.object = inliers_msg;
    ROS_INFO("[ClutterSegmenter:] service <clutseg_inliers> returning");
    return true;
  }


  ////////////////////////////////////////////////////////////////////////////////
  void callback (const sensor_msgs::PointCloud2ConstPtr& pc, const sensor_msgs::ImageConstPtr& im)
  {
    //1. Reading input data
    // get image
    query_image = bridge.imgMsgToCv(im, "passthrough");
    // get cloud
    pcl::fromROSMsg (*pc, query_cloud);
    
    //TODO: check how to convert from CameraInfo Msg to opencv_candidate::Camera
    // it shall work for Julius since the data is from the same sensor
    opencv_candidate::Camera camera = opencv_candidate::Camera(camera_info, opencv_candidate::Camera::TOD_YAML);

    //2. Recognizing
    clutseg::Query query(query_image, query_cloud);
    clutseg::Result result;
    sgm.recognize(query, result);

    // 3. Publish to topics
    if (result.guess_made) 
      { 
	// 3.1. Publish inlier cloud
	clutseg::PointCloudT inliers = result.refine_choice.inlierCloud;
	pcl::toROSMsg (inliers, inliers_msg);
	inliers_publisher.publish(inliers_msg);

	//3.2. Publish name of the recognized object
	std_msgs::String object_msg;
	object_msg.data = result.refine_choice.getObject()->name;
	object_publisher.publish(object_msg);

	// 3.3. Publish pose
	geometry_msgs::PoseStamped pose_msg;
	opencv_candidate::Pose pose = result.refine_choice.aligned_pose();
	cv::Mat_<float> rmat = pose.r<cv::Mat_<float> >();
	cv::Rodrigues(pose.r<cv::Mat_<float> >(), rmat);
	Eigen::Matrix3f rotation_matrix;
	cv::cv2eigen(rmat, rotation_matrix);
	Eigen::Quaternion<float> quaternion(rotation_matrix);
	pose_msg.header.frame_id = pc->header.frame_id;
	pose_msg.pose.orientation.x = quaternion.x();
	pose_msg.pose.orientation.y = quaternion.y();
	pose_msg.pose.orientation.z = quaternion.z();
	pose_msg.pose.orientation.w = quaternion.w();
	pose_publisher.publish(pose_msg);
      }

    {
      // Just for debugging
      cv::Mat hud = query_image.clone();
      if (result.guess_made) 
	{
	  clutseg::drawGuess(hud, result.refine_choice, camera, opencv_candidate::PoseRT());
	}
      cv_bridge::CvImage cv_hud;
      //cv_hud.encoding = sensor_msgs::image_encodings::BGR8;
      cv_hud.image = hud;
      // cv::imshow("hud", cv_hud.image);
      // cv::waitKey(-1);
      hud_publisher.publish(cv_hud.toImageMsg());
    } 
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "cloudsegmenter_node");
  ros::NodeHandle n("~");
  ClutterSegmenter cs(n);
  ros::spin ();
  return (0);
}


// void callback (const sensor_msgs::ImageConstPtr& im, const sensor_msgs::PointCloud2ConstPtr& pc)
// {

// }

// int main(int argc, char **argv) 
// {
//     ros::init(argc, argv, "clutsegmenter");
//     ros::NodeHandle n ("~");



//     ros::Rate loop_rate(0.1);



// while (ros::ok()) {

        // 2. Recognize
        // --------------------------------------------------------------------


//         
//         // --------------------------------------------------------------------
       
 


//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

