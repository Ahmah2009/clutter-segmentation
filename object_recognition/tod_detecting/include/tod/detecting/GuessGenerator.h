/*
 * PoseGenerator.h
 *
 *  Created on: Dec 15, 2010
 *      Author: alex
 */

#ifndef POSEGENERATOR_H_
#define POSEGENERATOR_H_

#include <opencv2/opencv.hpp>
#include "posest/pnp_ransac.h"
#include <tod/core/TexturedObject.h>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

namespace tod
{
enum RecognitionMode
{
  TOD = 0,
  KINECT
};

class Guess
{
public:
  virtual ~Guess();

  Guess()
  {
  }
  Guess(const cv::Ptr<TexturedObject>& object, const PoseRT& rt, const cv::Mat& queryImage);

  std::vector<int> inliers;
  std::vector<cv::Point2f> imagePoints;

  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> projectedPoints;

  int imageIndex;
  float stddev;

  const PoseRT& pose() const;

  const cv::Ptr<TexturedObject> getObject() const;
  void setPose(const PoseRT& pose);

  /** \brief draw the guess -
   *    0 draws the reprojection
   *    1 the correspondence;
   *
   */
  void draw(cv::Mat& out, int flags = 0, const std::string& directory = ".") const;

private:
  cv::Ptr<TexturedObject> object_;
  PoseRT pose_;
  cv::Mat query_; //!< the image that generated the imagePoints

};

struct GuessGeneratorParameters : public Serializable
{
  void write(cv::FileStorage& fs) const;

  void read(const cv::FileNode& fn);

  int minClusterSize;
  float minStddevFactor;
  int minInliersCount;
  int ransacIterationsCount;
  float maxProjectionError;
  float descriptorDistanceThreshold;

  static const std::string YAML_NODE_NAME;
};

class GuessGenerator
{
public:
  GuessGenerator(GuessGeneratorParameters params_);
  void calculateGuesses(const cv::Ptr<TexturedObject>& object, const std::vector<std::vector<int> >& clusterIndices,
                        const std::vector<cv::DMatch>& matches, const KeypointVector& keypoints, const cv::Mat& image,
                        std::vector<Guess>& guesses);
  void calculateGuesses(const cv::Ptr<TexturedObject>& object, const std::vector<cv::DMatch>& matches,
                        const KeypointVector& keypoints, const cv::Mat& image, std::vector<Guess>& guesses);
private:
  GuessGeneratorParameters params;
};

}

#endif /* POSEGENERATOR_H_ */
