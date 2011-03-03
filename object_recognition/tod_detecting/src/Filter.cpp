/*
 * Filter.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#include <tod/detecting/Filter.h>
#include <tod/detecting/Tools.h>
#include <tod/detecting/HCluster.h>

using namespace tod;
using namespace std;
using namespace cv;

Filter::Filter(const cv::Ptr<TexturedObject>& texturedObject):object(texturedObject)
{
}

StdDevFilter::StdDevFilter(const Ptr<TexturedObject>& texturedObject, const GuessGeneratorParameters& generatorParams):
    Filter(texturedObject), params(generatorParams)
{
}

StdDevFilter::~StdDevFilter()
{
}

void StdDevFilter::filterGuesses(vector<Guess>& guesses)
{
  for (vector<Guess>::iterator guessIt = guesses.begin(); guessIt != guesses.end();)
  {
    cout << "Factor: " << guessIt->stddev/object->stddev << endl << "Stddev: " << object->stddev  << endl << "GuessStd: " << guessIt->stddev << endl;
    if ((int)guessIt->inliers.size() < params.minInliersCount)
    {
      guessIt = guesses.erase(guessIt);
    }
    else if (guessIt->stddev < object->stddev  * params.minStddevFactor)
    {
      guessIt = guesses.erase(guessIt);
    }
    else
    {
      guessIt++;
    }
  }
}

OverlappingFilter::OverlappingFilter(const Ptr<TexturedObject>& texturedObject, const cv::Size& size_):
    Filter(texturedObject), size(size_)
{
}

OverlappingFilter::~OverlappingFilter()
{
}

void OverlappingFilter::initProjectedPoints(std::vector<Guess>& guesses)
{
  for (size_t i = 0; i < guesses.size(); i++)
  {
    vector<Point3f> points;
    vector<Point2f> projectedPoints;
    int imageIndex = guesses[i].imageIndex;
    if (imageIndex >= 0)
    {
      for (size_t point = 0; point < object->observations[imageIndex].cloud().size(); point++)
      {
        points.push_back(object->observations[imageIndex].cloud()[point]);
      }
      projectPoints(Mat(points), guesses[i].pose().rvec, guesses[i].pose().tvec,
                    object->observations[imageIndex].camera().K,
                    object->observations[imageIndex].camera().D, projectedPoints);
    }
    else
    {
        int cloudIndex = 0;
        PoseRT pose = Tools::invert(object->observations[cloudIndex].features().camera.pose);
        project3dPoints(object->observations[cloudIndex].cloud(), pose.rvec, pose.tvec, points);
        projectPoints(Mat(points), guesses[i].pose().rvec, guesses[i].pose().tvec,
                             object->observations[cloudIndex].camera().K,
                             object->observations[cloudIndex].camera().D, projectedPoints);
    }
    vector<Point2f>::iterator point;
    for (point = projectedPoints.begin(); point != projectedPoints.end();)
    {
      if (point->x < 0 || point->y < 0 || point->x >= size.width || point->y >= size.height)
      {
        point = projectedPoints.erase(point);
      }
      else
      {
        point++;
      }
    }
    guesses[i].projectedPoints = projectedPoints;
 }
}

void OverlappingFilter::filterGuesses(vector<Guess>& guesses)
{
  initProjectedPoints(guesses);

  vector<HCluster> clusters;
  for (vector<Guess>::const_iterator it = guesses.begin(); it != guesses.end(); it++)
  {
     HCluster cluster(*it, it->projectedPoints, it->inliers.size());
     clusters.push_back(cluster);
  }

  const float minCover = 0.5;
  HCluster::filterClusters(clusters, minCover);

  guesses.clear();
  for (vector<HCluster>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    guesses.push_back(it->guess);
  }
}


