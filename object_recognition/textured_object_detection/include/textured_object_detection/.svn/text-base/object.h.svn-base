#if !defined(_OBJECT_H)
#define _OBJECT_H

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <image_geometry/pinhole_camera_model.h>

//#define PAS_FEATURES
//#undef TEST_TO_TRAIN
//#define TRAIN_TO_TEST_M2M
//#define HOUGH_TEST

#ifdef PAS_FEATURES
#include "textured_object_detection/PASDetection.hpp"
#endif

//#define CHAMFER_PLUS_TEXTURE

class Object
{
  public:
    int imagesNum;
    std::string name;
    std::vector<cv::Mat> images;
    std::vector< std::vector<cv::KeyPoint> > keypoints;
    std::vector< std::vector<uint16_t> > pointsIndices;
    std::vector<cv::Mat> descriptors;
    std::vector< pcl::PointCloud<pcl::PointXYZ> > clouds;
    image_geometry::PinholeCameraModel cam_model;
    float stddev; // stddev of a 3d point cloud

#ifdef CHESSBOARD
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
#endif

#ifdef CHAMFER_PLUS_TEXTURE
    std::vector< std::vector<cv::KeyPoint> > edgels;
    std::vector< std::vector<uint16_t> > edgelsIndices;
    std::vector<cv::Mat> edgelsMaskImages;
#endif

//#ifdef PAS_FEATURES
    std::vector<cv::Mat> csnImages;
//#endif
#ifdef PAS_FEATURES
    std::vector<ContourSegmentNetwork> csns;
#endif

    Object(const std::string _name, const std::string _imagesPath, int _imagesNum, const image_geometry::PinholeCameraModel* camModel, 
		const cv::FeatureDetector* detector, const cv::DescriptorExtractor* descriptor, bool isRecalcDescriptors);
	void addImage(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>& cloud, const image_geometry::PinholeCameraModel* cam_model,
		    	  const cv::FeatureDetector* detector, const cv::DescriptorExtractor* descriptor, const std::string& path,
		    	  const std::string& index, bool isRecalcDescriptors);
    virtual ~Object();
    
  private:
	void filterKeyPoints(int img_ind, cv::Mat& mask, bool replaceByNearestPoint=false);
	void filterPoints(const cv::Mat &image, std::vector<cv::KeyPoint > &points, cv::Mat &mask, bool filterBoundaryPoints, bool replaceByNearestPoint);
	void setPointIndices(int img_ind, const image_geometry::PinholeCameraModel* camModel, cv::Mat &mask);
	void computeDescriptors(int img_ind, cv::Mat& mask, const cv::FeatureDetector* detector, const cv::DescriptorExtractor* descriptor);

#ifdef CHAMFER_PLUS_TEXTURE
	static double GET_CANNY_THRESHOLD_1(){ return 120; }
	static double GET_CANNY_THRESHOLD_2(){ return 80; }
#endif
};

float computeStddev(const std::vector<cv::Point3f>& points);
float computeStddevWithFilter(const std::vector<cv::Point3f>& points);
void filterOutlierPoints(std::vector<cv::Point3f>& points, float quantile = 0.9);


#endif
