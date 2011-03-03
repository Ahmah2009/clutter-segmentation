#if !defined(_TRAININGSET_H)
#define _TRAININGSET_H

#include "textured_object_detection/object.h"
#include "textured_object_detection/ts_parameters.h"
#include <ros/ros.h>

#include <list>
#include <algorithm>
#include <fstream>

#ifdef CHAMFER_PLUS_TEXTURE
#include "textured_object_detection/chamfer.hpp"
#endif
struct ObjectInfo
{
  int objectId;
  std::string objectName;
  cv::Mat rvec;
  cv::Mat tvec;
  int imgIdx;

  ObjectInfo()
  {
    imgIdx = 0;
    objectId = 0;
    objectName = "";
    rvec.create(3, 1, CV_64FC1);
    rvec = cv::Scalar(0.0);
    tvec.create(3, 1, CV_64FC1);
    tvec = cv::Scalar(0.0);
  }

  ObjectInfo & operator =(const ObjectInfo& object)
  {
    if (this != &object)
    {
      objectId = object.objectId;
      objectName = object.objectName;
      object.rvec.copyTo(rvec);
      object.tvec.copyTo(tvec);
      imgIdx = object.imgIdx;
    }
    return *this;
  }


  ObjectInfo(const ObjectInfo& object)
  {
    objectId = object.objectId;
    objectName = object.objectName;
    object.rvec.copyTo(rvec);
    object.tvec.copyTo(tvec);
    imgIdx = object.imgIdx;
  }

  //old format
  static void saveObjects(cv::FileStorage& fs, const std::vector<ObjectInfo>& objects);
  void write(cv::FileStorage& fs) const;

  //new format
  void write(cv::FileStorage& fs, int image_index, int objectIndex) const;
  static void saveObjects(cv::FileStorage& fs, int image_index, const std::vector<ObjectInfo>& objects, int& totalObjectsCount);
  static void loadObjects(cv::FileNode& objectsNode, std::vector<int>& image_indexes, std::vector<ObjectInfo>& objects);
};

class TrainingSet
{
public:  
  //draw inliers in matches between test image and train base
  bool isDrawInliers;
  //draw all matches
  bool isDrawCorrespondence;
  //draw object's projection to image
  bool isDrawProjection;
  //draw clusters
  bool isDrawClusters;
  //print some useful data
  bool isPrintResults;
  // print timing
  bool isPrintTiming;
  // save the images to file as opposed to showing them
  bool isSaveImages;
  // test name used to form image filenames
  std::string testName;
  // says if we have use highgui windows for visualization
  bool isNode;

  //train base contains images and point clouds for every objects
  std::vector<Object*> objects;
  int objNum;                   // number of objects (not number of views)

  cv::Mat bbProjImg;
  cv::Mat projImg;

  TSParameters params;

  class PoseGuess
  {
  public:
    cv::Mat rvec;
    cv::Mat tvec;
    std::vector<int> inliers;
    int img_idx;
    std::vector<cv::Point2f> objPoints;
    std::vector<cv::Point2f> imagePoints;
    std::vector<int> imageIndexes;
    int fineInliersCount;
    float stddev;

    std::vector<cv::Point2f> projectedPoints;
  };

  //camera info for left camera in stereo pair
  //train base contains images from left camera and point cloud

  TrainingSet(std::string dir = "", std::string configPath = "");

  virtual ~TrainingSet();

  //main function for recognizing objects
  //now it returns only names of found objects
  //img - test image
  //objects - vector of found objects, if size() == 0 => package don't find any known object
  //points_pub - publisher, that publish pose of found objects
  void recognize(const cv::Mat img, std::vector<ObjectInfo>& objects, const ros::Publisher &points_pub);
  
  void findPoseGuesses(Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::DMatch>& obj_matches, const std::vector<std::vector<int> >& indices,
                                    const cv::Mat& img, std::vector<PoseGuess>& poseGuesses);
  void refinePoses(const Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::DMatch>& matches,
                   const cv::Mat& img, std::vector<PoseGuess>& guesses);
  void filterOverlappingPoses(const std::vector<cv::Point2f>& imagePoints, std::vector<PoseGuess>& guesses);

  void drawInliers(const std::string test, std::string train);

  void initTestCamModel(std::string filename);

  //publish points
  void publishPoints(const std::vector<cv::Point3f>& points, const ros::Publisher &points_pub, const int index);
  void publishPoints(const std::vector<cv::Point3f>& points, const ros::Publisher &points_pub, const std::vector<int>& indices);

  const Object* getObject(size_t index) const {return index < objects.size() ? objects[index] : NULL;};
  const cv::Mat& getCameraMatrix() const {return cameraMatrix;};
  const cv::Mat& getDistCoeffs() const {return distCoeffs;};
  void setTestCameraInfo(const image_geometry::PinholeCameraModel& cam_model);
  void setTestCameraInfo(sensor_msgs::CameraInfoConstPtr cameraInfo);


protected:
  int maxImgNum;                // max number of views per object
  //scale factor for cvShowImage in drawing functions
  std::vector<cv::Scalar> colors;
  //detector for finding keypoints
  cv::Ptr<cv::FeatureDetector> detector;
  //descriptor
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
  //we merge descriptors of every image in train base to one big matrix
  //for matching test image to all train images simultaneously
  cv::Mat descriptors;
  //keypoint_indices - row indexes in descriptors matrix of corresponding train image for every descriptor
  //in descriptors matrix
  std::vector<int> keypoint_indices;
  //image index for every descriptor
  std::vector<int> img_indices;
  //object index for every descriptor
  std::vector<int> obj_indices;
  //flann index for matching
  cv::flann::Index* flann_index;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
    
#ifdef PAS_FEATURES
  cv::Mat codebook;
  cv::Mat idf;
  static double GET_DISSIMILARITY_THRESHOLD(){ return 2.; }
#endif
  
  void init();
  //read CameraInfo from txt file
  void readCamModel(std::string filename, sensor_msgs::CameraInfo& ci, image_geometry::PinholeCameraModel*& camModel);
  //read objects from disc
  void readObjects(std::string filename, std::string dir);
  //merge descriptors into one big matrix
  void concatenateMatrices();
  //draw matches
  void drawCorrespondence(const cv::Mat& img, const std::vector<cv::KeyPoint>& keypoints,
                         const std::vector<cv::DMatch>& matches);
  //draw matches if we match every train image to test image
  void drawCorrespondencesOneTrainImage(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::KeyPoint>& keypoints2, const std::vector<int>& matches,
                                        const std::vector<int>& inlierIndices);
  //draw inliers
  void drawInliers(const cv::Mat img, int obj_ind, int img_ind, const std::vector<cv::Point2f> objPoints,
                   const std::vector<cv::Point2f> imagePoints, const std::vector<int>& inliers, bool isCSN=false, int rect_ind=-1,  cv::Rect rect=cv::Rect());
  //draw projection of found objects
  void drawProjectedPoints(const cv::Mat img, int obj_ind, int img_ind, const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> objPoints,
                           const std::vector<cv::Point2f> imagePoints, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const cv::Mat &rvec, const cv::Mat& tvec, bool isCSN=false, int rect_ind=-1, cv::Rect rect=cv::Rect() );
  //draw clusters
  void drawClusters(const cv::Mat img, const std::vector<cv::Point2f>& imagePoints,  const std::vector< std::vector<int> >& indixes);
  //hierarchical clustering
  void clusterPoints(const std::vector<cv::Point2f>& imagePoints, const std::vector<int>& imageIndexes, float maxDist, std::vector< std::vector<int> >& clusters);
  //detect keypoints on test image, compute descriptors and match them
  void detectAndMatchPoints(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
  //create detector
  cv::FeatureDetector* createDetector(const std::string& detectorType);
  //create descriptor
  cv::DescriptorExtractor* createDescriptorExtractor(const std::string& descriptorType);
  //create matcher
  cv::DescriptorMatcher* createDescriptorMatcher(const std::string& matcherType);
  void printMatchingResults(const std::vector<cv::DMatch>& matches);
  void drawProjectedPoints(const std::vector<cv::Point3f>& rotated_object_points, int objIndex);
  void nullCloudFilter(std::vector<PoseGuess>& guesses, const cv::Mat& img, const Object* object, const std::vector<cv::Point2f> imgPoints);
};


struct HCluster
{
  TrainingSet::PoseGuess guess;
  std::vector<cv::Point2f> projectedPoints;
  std::vector<cv::Point2f> hull;
  int inlierCount;

  HCluster(const HCluster& cluster) : guess(cluster.guess), inlierCount(cluster.inlierCount)
  {
    setProjectedPoints(cluster.projectedPoints);
//    indices = cluster.indices;
//    updateHull();
  }

  HCluster(const TrainingSet::PoseGuess& _guess, const std::vector<cv::Point2f>& points, int _inlierCount = -1) :
    guess(_guess), inlierCount(_inlierCount)
  {
    setProjectedPoints(points);
  }

  float oneWayCover(const HCluster& cluster) const
  {
    cv::Mat _hull(hull);
    int sum = 0;
    for(size_t i = 0; i < cluster.projectedPoints.size(); i++)
    {
      float dist = cv::pointPolygonTest(_hull, cluster.projectedPoints[i], true);
      if(dist > 0)
      {
        sum++;
#if 0
        cv::Mat drawImg = cv::Mat::zeros(2500, 2000, CV_8UC3);

        for(size_t k = 0; k < hull.size(); k++)
        {
          int idx1 = k;
          int idx2 = (k+1)%hull.size();

          cv::line(drawImg, cv::Point(hull[idx1].x, hull[idx1].y), cv::Point(hull[idx2].x, hull[idx2].y), cv::Scalar(0, 0, 255));
        }

        for(size_t k = 0; k < indices.size(); k++)
        {
          cv::Point2f p = points[0][indices[k]];
          cv::circle(drawImg, cv::Point(p.x, p.y), 5, cv::Scalar(0, 0, 255));
        }

//        for(size_t k = 0; k < clusters[j].indices.size(); k++)
        size_t k = i;
        {
          cv::Point2f p = cluster.points[0][cluster.indices[k]];
          cv::circle(drawImg, cv::Point(p.x, p.y), 5, cv::Scalar(255, 0, 0));
        }

        cv::namedWindow("1", 1);
        cv::Mat drawImgS;
        resize(drawImg, drawImgS, cv::Size(drawImg.cols*0.4, drawImg.rows*0.4));
        cv::imshow("1", drawImgS);
        cv::waitKey(0);
#endif
      }
    }
    printf("pointPolygonTest returns %d points, total points %d\n", sum, (int)cluster.projectedPoints.size());

    return float(sum);
  }

  float cover(const HCluster& cluster) const
  {
    float dist1 = oneWayCover(cluster);
    float dist2 = cluster.oneWayCover(*this);
    return (dist1 + dist2)/(projectedPoints.size() + cluster.projectedPoints.size());
  }

  void updateHull()
  {
    cv::convexHull(cv::Mat(projectedPoints), hull);

    std::vector<cv::Point2f> hullApprox;
    cv::approxPolyDP(cv::Mat(hull), hullApprox, 1.0, true);
    hull = hullApprox;
  }

  void setProjectedPoints(const std::vector<cv::Point2f>& points)
  {
    //apply found R and T to point cloud
    projectedPoints = points;
    updateHull();
  }

  static bool pred(const HCluster& c1, const HCluster& c2)
  {
    return c1.inlierCount > c2.inlierCount;
  }
};

/*
//hierarchical clustering of points from test image than matched to different images of one object
//imagePoints - coord of points
//imageIndexes - image indexes in train base (we prohibit points matched to different images in one cluster, so distance between them is infinity)
//maxDist - max distance for clustering
//clusterIndixes - result indexes for every cluster
template<typename Cluster>
void hierarchicalCluster(std::list<Cluster>& clusters, float maxDist)
{
  while(1)
  {
    std::list<Cluster>::iterator minIt1 = clusters.end(), minIt2 = clusters.end();
    float minDist = 1e10;
    for(std::list<Cluster>::iterator it1 = clusters.begin(); it1 != clusters.end(); it1++)
    {
      for(std::list<Cluster>::iterator it2 = clusters.begin(); it2 != clusters.end(); it2++)
      {
        float dist = it1->distance(*it2);
        if(it1 != it2 && dist < minDist)
        {
          minDist = dist;
          minIt1 = it1;
          minIt2 = it2;
        }
      }
    }

    if(mini == -1 || minDist > maxDist) break;

    // merge two clusters
    it1->merge(*it2);
    clusters.erase(it2);
  }
}
*/

//template<class T>
inline void filterClusters(std::vector<HCluster>& clusters, float minCover, cv::Mat& img)
{
  std::sort(clusters.begin(), clusters.end(), HCluster::pred);
  std::vector<int> flags;
  flags.assign(clusters.size(), 1);

  for(size_t i = 0; i < clusters.size(); i++)
  {
    if(flags[i] == 0) continue;

    for(size_t j = i + 1; j < clusters.size(); j++)
    {
      if(flags[j] == 0) continue;

      printf("Overlap between %d and %d is %f\n", int(i), int(j),
             clusters[i].oneWayCover(clusters[j])/clusters[j].projectedPoints.size());
      if(clusters[i].oneWayCover(clusters[j]) > minCover*clusters[j].projectedPoints.size())
      {
        flags[j] = 0;
        printf("Removing cluster %d, inliers %d, in favor of cluster %d, inliers %d\n",
               (int)j, (int)clusters[j].projectedPoints.size(), (int)i, (int)clusters[i].projectedPoints.size());
      }

#if 0
      cv::Mat drawImg;// = cv::Mat::zeros(2000, 2000, CV_8U);
      img.copyTo(drawImg);

      for(size_t k = 0; k < clusters[i].hull.size(); k++)
      {
        int idx1 = k;
        int idx2 = (k+1)%clusters[i].hull.size();

        cv::line(drawImg, cv::Point(clusters[i].hull[idx1].x, clusters[i].hull[idx1].y), cv::Point(clusters[i].hull[idx2].x, clusters[i].hull[idx2].y), cv::Scalar(0, 0, 255));
      }

      for(size_t k = 0; k < clusters[i].projectedPoints.size(); k++)
      {
        cv::circle(drawImg, clusters[i].projectedPoints[k], 5, cv::Scalar(0, 0, 255));
      }

      for(size_t k = 0; k < clusters[j].projectedPoints.size(); k++)
      {
        cv::Point2f p = clusters[j].projectedPoints[k];
        cv::circle(drawImg, cv::Point(p.x, p.y), 5, cv::Scalar(255, 0, 0));
      }

      cv::namedWindow("1", 1);
      cv::Mat drawImgS;
      resize(drawImg, drawImgS, cv::Size(drawImg.cols*0.3, drawImg.rows*0.3));
      cv::imshow("1", drawImgS);
      cv::waitKey(0);
#endif
    }
  }

  std::vector<HCluster> filtered;
  for(size_t i = 0; i < clusters.size(); i++)
  {
    if(flags[i] == 1)
    {
      filtered.push_back(clusters[i]);
      printf("Cluster %d -> %d\n", i, filtered.size() - 1);
    }
  }

  clusters = filtered;
}

#endif
