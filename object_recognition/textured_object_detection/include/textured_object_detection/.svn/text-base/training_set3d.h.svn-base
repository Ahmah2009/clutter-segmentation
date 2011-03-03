#ifndef TRAINING_SET3D_H_
#define TRAINING_SET3D_H_

#include "cv.h"
#include "highgui.h"

#include "textured_object_detection/training_set.h"

class TrainingSet3d: public TrainingSet
{
public:
  TrainingSet3d(std::string dir = "", std::string configPath = "");
  void recognize(const cv::Mat& img, const cv::Mat& mask, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                 std::vector<ObjectInfo>& objects, const ros::Publisher &points_pub);
  void findPoseGuesses(Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& test3dPoints,
                       const std::vector<cv::DMatch>& obj_matches, const std::vector<std::vector<int> >& indices,
                                    const cv::Mat& img, std::vector<PoseGuess>& poseGuesses);
  void refinePoses(const Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& test3dPoints, const std::vector<cv::DMatch>& matches,
                   const cv::Mat& img, std::vector<PoseGuess>& guesses);
  void detectAndMatchPoints(const cv::Mat& img, const cv::Mat& mask, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
  virtual ~TrainingSet3d();
};

#endif /* TRAINING_SET3D_H_ */
