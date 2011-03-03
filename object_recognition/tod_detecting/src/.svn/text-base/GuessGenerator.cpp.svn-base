/*
 * PoseGenerator.cpp
 *
 *  Created on: Dec 15, 2010
 *      Author: alex
 */
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Tools.h>
#include <boost/foreach.hpp>
#include <tod/training/pose.h>
#define foreach BOOST_FOREACH

using namespace std;
using namespace cv;
using namespace tod;

const string GuessGeneratorParameters::YAML_NODE_NAME = "GuessParameters";

void GuessGeneratorParameters::write(cv::FileStorage& fs) const
 {
   cvWriteComment(*fs, "GuessParameters", 0);
   fs << "{";
   fs << "min_cluster_size" << minClusterSize;
   fs << "min_inliers_count" << minInliersCount;
   fs << "ransac_iterations_count" << ransacIterationsCount;
   fs << "max_projection_error" << maxProjectionError;
   fs << "descriptor_distance_threshold" << descriptorDistanceThreshold;
   fs << "min_stddev_factor" << minStddevFactor;
   fs << "}";
}

 void GuessGeneratorParameters::read(const cv::FileNode& fn)
 {
   minClusterSize = (int)fn["min_cluster_size"];
   minInliersCount = (int)fn["min_inliers_count"];
   ransacIterationsCount = (int)fn["ransac_iterations_count"];
   maxProjectionError = (float)fn["max_projection_error"];
   descriptorDistanceThreshold = (float)fn["descriptor_distance_threshold"];
   minStddevFactor = (float)fn["min_stddev_factor"];
 }

Guess::~Guess()
{
}
Guess::Guess(const cv::Ptr<TexturedObject>& object, const PoseRT& rt, const cv::Mat& queryImage) :
      object_(object), pose_(rt), query_(queryImage)
{
}

const PoseRT& Guess::pose() const
{
  return pose_;
}

const cv::Ptr<TexturedObject> Guess::getObject() const
{
  return object_;
}

void Guess::setPose(const PoseRT& pose)
{
  pose_ = pose;
}

void Guess::draw(cv::Mat& out, int flags, const std::string& directory) const
{
  if (flags == 0)
  {
    if (query_.channels() != 3)
    {
      if (out.empty())
        cvtColor(query_, out, CV_GRAY2RGB);
    }
    else if (out.empty())
      query_.copyTo(out);

    vector<Point2f> projectedPoints;

    if (imageIndex >= 0)
      projectPoints(Mat(object_->observations[imageIndex].cloud()), pose().rvec, pose().tvec,
                    object_->observations[imageIndex].camera().K, object_->observations[imageIndex].camera().D,
                    projectedPoints);
    else
    {
      vector<Point3f> cloud, rotatedPoints;
      int cloudIndex = 0;
      PoseRT inverted = Tools::invert(object_->observations[cloudIndex].features().camera.pose);
      project3dPoints(Mat(object_->observations[cloudIndex].cloud()), inverted.rvec, inverted.tvec, cloud);
      project3dPoints(Mat(cloud), pose().rvec, pose().tvec, rotatedPoints);
      Tools::filterOutlierPoints(rotatedPoints, 0.9);
      if (rotatedPoints.size() < 7)
        return;
      projectPoints(Mat(rotatedPoints), Mat(3, 1, CV_64F, Scalar(0.0)), Mat(3, 1, CV_64F, Scalar(0.0)),
                    object_->observations[cloudIndex].camera().K, object_->observations[cloudIndex].camera().D, projectedPoints);
    }

    Point2f topright(0, 0);
    vector<Point2f>::iterator point;
    for (point = projectedPoints.begin(); point != projectedPoints.end();)
    {
      if (point->x < 0 || point->y < 0 || point->x >= out.cols || point->y  >= out.rows)
      {
        point = projectedPoints.erase(point);
      }
      else
      {
        circle(out, *point, 3, Scalar(255, 0, 0), 1);
        topright += *point;
        point++;
      }
    }
    if (projectedPoints.size() > 0)
    {
      vector<Point2f> hull;
      convexHull(Mat(projectedPoints), hull);

      vector<Point> ihull;
      for (size_t i = 0; i < hull.size(); i++)
      {
        ihull.push_back(Point(hull[i].x, hull[i].y));
      }

      vector<vector<Point> > _hull;
      _hull.push_back(ihull);

      drawContours(out, _hull, -1, Scalar(255, 0, 0), 1);
    }

    topright.x /= projectedPoints.size();
    topright.y /= projectedPoints.size();
    putText(out, object_->name, topright, CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 255), 2, CV_AA, false);

    if (imageIndex > 0)
    {
      PoseRT frame_pose;
      PoseRT camera_pose = object_->observations[imageIndex].camera().pose;
      cv::composeRT(camera_pose.rvec, camera_pose.tvec, pose().rvec, pose().tvec, frame_pose.rvec, frame_pose.tvec);
      PoseDrawer(out, object_->observations[imageIndex].camera().K, frame_pose);
    }
    else
    {
      int cloudIndex = 0;
      PoseRT inverted = Tools::invert(object_->observations[cloudIndex].features().camera.pose);
      PoseRT object_pose;
      cv::composeRT(inverted.rvec, inverted.tvec, pose().rvec, pose().tvec, object_pose.rvec, object_pose.tvec);
      PoseRT frame_pose;
      PoseRT camera_pose = object_->observations[cloudIndex].camera().pose;
      cv::composeRT(camera_pose.rvec, camera_pose.tvec, object_pose.rvec, object_pose.tvec, frame_pose.rvec, frame_pose.tvec);
      PoseDrawer(out, object_->observations[cloudIndex].camera().K, frame_pose);
    }
  }
  else if (flags == 1)
  {
    if (imageIndex > 0)
    {
      Features3d f3d = object_->observations[imageIndex];
      Features2d f2d = f3d.features();
      if (f2d.image.empty())
      {
        string filename = directory + "/" + object_->name + "/" + f2d.image_name;
        f2d.image = imread(filename, 0);
      }

      vector<Point2f> oppoints;
      projectPoints(Mat(objectPoints), Mat::zeros(3, 1, CV_64F), Mat::zeros(3, 1, CV_64F),
                    object_->observations[imageIndex].camera().K, object_->observations[imageIndex].camera().D, oppoints);

      vector<DMatch> matches;
      vector<char> mask(objectPoints.size(), 0);
      KeypointVector kpts_train, kpts_query;
      // KeyPoint::convert(imagePoints,kpts_query,0,0,0,0);
      // KeyPoint::convert(oppoints,kpts_train,0,0,0,0);
      for (size_t i = 0; i < objectPoints.size(); i++)
      {
        DMatch m(i, i, 0);
        matches.push_back(m);
        kpts_train.push_back(KeyPoint(oppoints[i].x, oppoints[i].y, 0, 0, 0, 0));
        kpts_query.push_back(KeyPoint(imagePoints[i].x, imagePoints[i].y, 0, 0, 0, 0));
      }
      foreach(int inlier,inliers)
            {
              mask[inlier] = 1;
            }

      cv::drawMatches(f2d.image, kpts_train, query_, kpts_query, matches, out, Scalar(255, 0, 0), Scalar(0, 0, 255), mask);
    }
  }
}
GuessGenerator::GuessGenerator(GuessGeneratorParameters params_)
{
  params = params_;
}

void GuessGenerator::calculateGuesses(const cv::Ptr<TexturedObject>& object, const std::vector<cv::DMatch>& matches,
                                      const KeypointVector& keypoints, const cv::Mat& image,
                                      std::vector<Guess>& guesses)
{
  guesses.clear();

  typedef std::map<int, vector<DMatch> > ObservationMap;
  ObservationMap observation_mapping;
  foreach(const DMatch& m, matches)
  {
    observation_mapping[m.imgIdx].push_back(m); // collect all observation matches in order
  }
  vector<Point2f> imagePoints;
  vector<Point3f> objectPoints;
  foreach(const ObservationMap::value_type& kp, observation_mapping)
  {
    int observation = kp.first;
    vector<DMatch> o_matches = kp.second;

    //todo add prosac
//    sort(o_matches.begin(), o_matches.end());
//    o_matches.resize(o_matches.size() * params.descriptorDistanceThreshold);

    if (int(o_matches.size()) < params.minClusterSize)
          continue;

    imagePoints.clear(); //try to recycle some memory...
    objectPoints.clear();

    const Features3d& f3d = object->observations[observation];

    foreach(const DMatch& m, o_matches)
    {
      objectPoints.push_back(f3d.cloud()[m.trainIdx]);
      imagePoints.push_back(keypoints[m.queryIdx].pt);
    }
    /*cv::Mat out;
    cv::Mat test_image = cv::imread(f3d.features().image_name);
    cv::drawMatches(image, keypoints, f3d.features().image, f3d.features().keypoints, o_matches, out, Scalar(255, 0, 0), Scalar(0, 0, 255));
    imshow("matches for ransac", out);
    waitKey(0);*/

    vector<int> inliers;
    Mat rvec, tvec;
    solvePnPRansac(objectPoints, imagePoints, f3d.camera().K, f3d.camera().D, rvec, tvec, false,
                   params.ransacIterationsCount, params.maxProjectionError, -1, &inliers);

    if ((int)inliers.size() > params.minInliersCount)
    {
      Guess guess(object, PoseRT(rvec, tvec), image);
      guess.inliers = inliers;
      guess.imagePoints = imagePoints;
      guess.objectPoints = objectPoints;
      guess.imageIndex = observation;
      guesses.push_back(guess);
    }
  }
}
void GuessGenerator::calculateGuesses(const Ptr<TexturedObject>& object, const vector<vector<int> >& clusterIndices,
                                      const vector<DMatch>& matches, const KeypointVector& keypoints, const Mat& image,
                                      vector<Guess>& guesses)
{
  foreach(const vector<int>& cluster, clusterIndices)
  {
    if ((int)cluster.size() < params.minClusterSize)
      continue;

    vector<int> activePoints;
    activePoints.assign(cluster.size(), 1);

    vector<Point2f> imagePoints;
    vector<Point3f> objectPoints;
    int observationIndex =  matches[cluster[0]].imgIdx;
    while (*std::max_element(activePoints.begin(), activePoints.end()) > 0)
    {
      imagePoints.clear();
      objectPoints.clear();

      vector<int> activePointsIndices;

      for (size_t clusterIndex = 0; clusterIndex < cluster.size(); clusterIndex++)
      {
        if (activePoints[clusterIndex] == 0)
          continue;

        activePointsIndices.push_back(clusterIndex);

        int matchIndex = cluster[clusterIndex];
        imagePoints.push_back(keypoints[matches[matchIndex].queryIdx].pt);

        objectPoints.push_back(object->observations[observationIndex].cloud()[matches[matchIndex].trainIdx]);

        /* int observationIndex =  matches[matchIndex].imgIdx;
            vector<Point3f> points, rotatedPoints;
            points.push_back(object->observations[observationIndex].cloud()[matches[matchIndex].trainIdx]);
         PoseRT inverted = Tools::invert(object->observations[observationIndex].camera().pose);
            project3dPoints(points, inverted.rvec, inverted.tvec, rotatedPoints);
            objectPoints.push_back(rotatedPoints[0]);*/
      }

      vector<int> inliers;
      Mat rvec, tvec;

            //TODO: we need camera parameters for query image
            solvePnPRansac(objectPoints, imagePoints, object->observations[0].camera().K,
                           object->observations[0].camera().D, rvec, tvec, false,
                           params.ransacIterationsCount, params.maxProjectionError, -1, &inliers);

      if ((int)inliers.size() > params.minInliersCount/2)
      {
        cout << inliers.size() << endl;
        Guess guess(object, PoseRT(rvec, tvec), image);
        guess.inliers = inliers;
        guess.imagePoints = imagePoints;
        guess.objectPoints = objectPoints;
        guess.imageIndex = observationIndex;
           guesses.push_back(guess);
        for (size_t m = 0; m < inliers.size(); m++)
        {
          activePoints[activePointsIndices[inliers[m]]] = 0; //removing the points for which the pose has been found
        }
      }
      else
      {
        break;
      }
    }
  }
}
