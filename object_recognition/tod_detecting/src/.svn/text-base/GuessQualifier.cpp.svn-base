/*
 * GuessQualifier.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <tod/detecting/Tools.h>
#include <tod/detecting/GuessQualifier.h>

using namespace tod;
using namespace cv;
using namespace std;

GuessQualifier::GuessQualifier(const Ptr<TexturedObject>& texturedObject, const vector<DMatch>& objectMatches,
                               const KeypointVector& testKeypoints, const GuessGeneratorParameters& generatorParams):
    object(texturedObject), matches(objectMatches), keypoints(testKeypoints), params(generatorParams), prevImageIdx(-2)
{
}

GuessQualifier::~GuessQualifier()
{
}

void GuessQualifier::initPointsVector(vector<Point2f>& imagePoints, vector<Point3f>& objectPoints, int imageIdx)
{
  if (imageIdx == prevImageIdx)
    return;

  if (imageIdx < 0)
  {
    imagePoints.resize(matches.size());
    objectPoints.resize(matches.size());
    for (size_t i = 0; i < matches.size(); i++)
    {
       imagePoints[i] = keypoints[matches[i].queryIdx].pt;
       vector<Point3f> point, convertedPoint;
       int observationIndex = matches[i].imgIdx;
       point.push_back(object->observations[observationIndex].cloud()[matches[i].trainIdx]);
       PoseRT pose = Tools::invert(object->observations[observationIndex].camera().pose);
       project3dPoints(point, pose.rvec, pose.tvec, convertedPoint);
       objectPoints[i] = convertedPoint[0];
    }
  }
  else
  {
    imagePoints.clear();
    objectPoints.clear();
    for (size_t i = 0; i < matches.size(); i++)
    {
      if (matches[i].imgIdx != imageIdx)
      {
        continue;
      }
      imagePoints.push_back(keypoints[matches[i].queryIdx].pt);
      objectPoints.push_back(object->observations[imageIdx].cloud()[matches[i].trainIdx]);
    }
  }
  prevImageIdx = imageIdx;
}

void GuessQualifier::clarifyGuess(Guess& guess, vector<Point2f>& imagePoints, vector<Point3f>& objectPoints)
{
  cout << "Guess: " << endl;
  cout << guess.pose().rvec << endl << guess.pose().tvec << " " << guess.inliers.size() << endl;

  // find inliers in global match array
  vector<Point2f> projectedPoints;
  projectPoints(Mat(objectPoints), guess.pose().rvec, guess.pose().tvec,
      object->observations[0].camera().K, object->observations[0].camera().D, projectedPoints);

  vector<Point2f> inlierImagePoints;
  vector<Point3f> inlierObjectPoints;
  vector<int> inlierIndices;
  for (size_t j = 0; j < imagePoints.size(); j++)
  {
    float dist = norm(imagePoints[j] - projectedPoints[j]);
    if (dist < params.maxProjectionError * 5)
    {
      inlierImagePoints.push_back(imagePoints[j]);
      inlierObjectPoints.push_back(objectPoints[j]);
      inlierIndices.push_back(j);
    }
  }
  if (inlierObjectPoints.size() >= 5)
  {
    vector<int> inliers;
    PoseRT pose = guess.pose();
    solvePnPRansac(inlierObjectPoints, inlierImagePoints,
        object->observations[0].camera().K, object->observations[0].camera().D,
        pose.rvec, pose.tvec, true, params.ransacIterationsCount * 10, params.maxProjectionError,
        params.minInliersCount, &inliers);

    guess.setPose(pose);
    guess.inliers.resize(inliers.size());
    // copy inlier indices into the global point array
    for (size_t j = 0; j < inliers.size(); j++)
    {
      guess.inliers[j] = inlierIndices[inliers[j]];
    }
    guess.imagePoints = imagePoints;
    guess.objectPoints = objectPoints;
  }
  float stddev = Tools::computeStdDev(inlierObjectPoints);
  guess.stddev = stddev;
  cout << "After clarifying: " << endl;
  cout << guess.pose().rvec << endl << guess.pose().tvec << " " << guess.inliers.size() << endl << endl;
}

void GuessQualifier::clarify(vector<Guess>& guesses)
{
  if (!guesses.size())
    return;

  vector<Point2f> imagePoints;
  vector<Point3f> objectPoints;

  foreach(Guess& guess, guesses)
  {
    //TODO: add sorting guesses on imageIdx
    initPointsVector(imagePoints, objectPoints, guess.imageIndex);
    clarifyGuess(guess, imagePoints, objectPoints);
  }
}
