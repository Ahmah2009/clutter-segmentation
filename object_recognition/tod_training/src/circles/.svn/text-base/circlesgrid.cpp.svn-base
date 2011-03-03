/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "circlesgrid.hpp"

using namespace cv;
using namespace std;
namespace tod{


bool findCirclesGrid( const Mat& image, Size patternSize,
                      vector<Point2f>& centers, int )
{
    Ptr<SimpleBlobDetector> detector = new SimpleBlobDetector();
    //Ptr<FeatureDetector> detector = new MserFeatureDetector();
    vector<KeyPoint> keypoints;
    detector->detect(image, keypoints);
    vector<Point2f> points;
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        points.push_back (keypoints[i].pt);
    }

    CirclesGridFinderParameters parameters;
    parameters.vertexPenalty = -0.6f;
    parameters.vertexGain = 1;
    parameters.existingVertexGain = 1000;
    parameters.edgeGain = 1;
    parameters.edgePenalty = -0.6f;

    const int attempts = 2;
    const int minHomographyPoints = 4;
    Mat H;
    for (int i = 0; i < attempts; i++)
    {
      centers.clear();
      CirclesGridFinder boxFinder(patternSize, points, parameters);
      bool isFound = false;
      try
      {
        isFound = boxFinder.findHoles();
      }
      catch (cv::Exception)
      {
      }
      cv::Mat drawImage;
      boxFinder.drawHoles(image,drawImage);
//      imshow("holes",drawImage);
//      waitKey();
      boxFinder.getHoles(centers);

      if (isFound)
      {
        if (i != 0)
        {
          Mat orgPointsMat;
          transform(Mat(centers), orgPointsMat, H.inv());
          convertPointsHomogeneous(orgPointsMat, centers);
        }

        return true;
      }

      if (i != attempts - 1)
      {
        if (int(centers.size()) < minHomographyPoints)
          break;
        H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
      }
    }

    return false;
}

/*
 *  SimpleBlobDetector
 */
SimpleBlobDetector::Params::Params()
{
  thresholdStep = 10;
  minThreshold = 50;
  maxThreshold = 220;
  maxCentersDist = 10;
  defaultKeypointSize = 1;
  minRepeatability = 2;
  computeRadius = true;
  filterByColor = true;
  blobColor = 0;

  isGrayscaleCentroid = false;
  centroidROIMargin = 2;

  filterByArea = true;
  minArea = 25;
  maxArea = 5000;

  filterByInertia = true;
  //minInertiaRatio = 0.6;
  minInertiaRatio = 0.1f;

  filterByConvexity = true;
  //minConvexity = 0.8;
  minConvexity = 0.95f;

  filterByCircularity = false;
  minCircularity = 0.8f;
}

SimpleBlobDetector::SimpleBlobDetector(const SimpleBlobDetector::Params &parameters) :
  params(parameters)
{
}

Point2d SimpleBlobDetector::computeGrayscaleCentroid(const Mat &image, const vector<Point> &contour) const
{
  Rect rect = boundingRect(Mat(contour));
  rect.x -= params.centroidROIMargin;
  rect.y -= params.centroidROIMargin;
  rect.width += 2 * params.centroidROIMargin;
  rect.height += 2 * params.centroidROIMargin;

  rect.x = rect.x < 0 ? 0 : rect.x;
  rect.y = rect.y < 0 ? 0 : rect.y;
  rect.width = rect.x + rect.width < image.cols ? rect.width : image.cols - rect.x;
  rect.height = rect.y + rect.height < image.rows ? rect.height : image.rows - rect.y;

  Mat roi = image(rect);
  assert( roi.type() == CV_8UC1 );

  Mat invRoi = 255 - roi;
  invRoi.convertTo(invRoi, CV_32FC1);
  invRoi = invRoi.mul(invRoi);

  Moments moms = moments(invRoi);

  Point2d tl = rect.tl();
  Point2d roiCentroid(moms.m10 / moms.m00, moms.m01 / moms.m00);

  Point2d centroid = tl + roiCentroid;
  return centroid;
}

void SimpleBlobDetector::findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, vector<Center> &centers) const
{
  centers.clear();

  vector<vector<Point> > contours;
  Mat tmpBinaryImage = binaryImage.clone();
  findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  //Mat keypointsImage;
  //cvtColor( binaryImage, keypointsImage, CV_GRAY2RGB );

  //Mat contoursImage;
  //cvtColor( binaryImage, contoursImage, CV_GRAY2RGB );
  //drawContours( contoursImage, contours, -1, Scalar(0,255,0) );
  //imshow("contours", contoursImage );

  for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    Center center;
    center.confidence = 1;
    Moments moms = moments(Mat(contours[contourIdx]));
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area > params.maxArea)
        continue;
    }

    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = arcLength(Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity)
        continue;
    }

    if (params.filterByInertia)
    {
      double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }

      if (ratio < params.minInertiaRatio)
        continue;

      center.confidence = ratio * ratio;
    }

    if (params.filterByConvexity)
    {
      vector<Point> hull;
      convexHull(Mat(contours[contourIdx]), hull);
      double area = contourArea(Mat(contours[contourIdx]));
      double hullArea = contourArea(Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity)
        continue;
    }

    if (params.isGrayscaleCentroid)
      center.location = computeGrayscaleCentroid(image, contours[contourIdx]);
    else
      center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

    if (params.filterByColor)
    {
      if (binaryImage.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) != params.blobColor)
        continue;
    }

    if (params.computeRadius)
    {
      vector<double> dists;
      for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
      {
        Point2d pt = contours[contourIdx][pointIdx];
        dists.push_back(norm(center.location - pt));
      }
      std::sort(dists.begin(), dists.end());
      center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    }

    centers.push_back(center);

    //circle( keypointsImage, center.location, 1, Scalar(0,0,255), 1 );
  }
  //imshow("bk", keypointsImage );
  //waitKey();
}

void SimpleBlobDetector::detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  keypoints.clear();
  Mat grayscaleImage;
  if (image.channels() == 3)
    cvtColor(image, grayscaleImage, CV_BGR2GRAY);
  else
    grayscaleImage = image;

  vector<vector<Center> > centers;
  for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
  {
    Mat binarizedImage;
    threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

    //Mat keypointsImage;
    //cvtColor( binarizedImage, keypointsImage, CV_GRAY2RGB );

    vector<Center> curCenters;
    findBlobs(grayscaleImage, binarizedImage, curCenters);
    for (size_t i = 0; i < curCenters.size(); i++)
    {
      //circle(keypointsImage, curCenters[i].location, 1, Scalar(0,0,255),-1);

      bool isNew = true;
      for (size_t j = 0; j < centers.size(); j++)
      {
        double dist = norm(centers[j][0].location - curCenters[i].location);
        if (params.computeRadius)
          isNew = dist >= centers[j][0].radius && dist >= curCenters[i].radius && dist >= params.maxCentersDist;
        else
          isNew = dist >= params.maxCentersDist;
        if (!isNew)
        {
          centers[j].push_back(curCenters[i]);
          //          if( centers[j][0].radius < centers[j][ centers[j].size()-1 ].radius )
          //          {
          //            std::swap( centers[j][0], centers[j][ centers[j].size()-1 ] );
          //          }
          break;
        }
      }
      if (isNew)
      {
        centers.push_back(vector<Center> (1, curCenters[i]));
      }
    }
    //imshow("binarized", keypointsImage );
    //waitKey();
  }

  for (size_t i = 0; i < centers.size(); i++)
  {
    if (centers[i].size() < params.minRepeatability)
      continue;
    Point2d sumPoint(0, 0);
    double normalizer = 0;
    for (size_t j = 0; j < centers[i].size(); j++)
    {
      sumPoint += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sumPoint *= (1. / normalizer);
    KeyPoint kpt(sumPoint, params.defaultKeypointSize);
    keypoints.push_back(kpt);
  }
}

Graph::Graph(int n)
{
  for (int i = 0; i < n; i++)
  {
    addVertex(i);
  }
}

bool Graph::doesVertexExist(int id) const
{
  return (vertices.find(id) != vertices.end());
}

void Graph::addVertex(int id)
{
  assert( !doesVertexExist( id ) );

  vertices.insert(pair<int, Vertex> (id, Vertex()));
}

void Graph::addEdge(int id1, int id2)
{
  assert( doesVertexExist( id1 ) );
  assert( doesVertexExist( id2 ) );

  vertices[id1].neighbors.insert(id2);
  vertices[id2].neighbors.insert(id1);
}

bool Graph::areVerticesAdjacent(int id1, int id2) const
{
  assert( doesVertexExist( id1 ) );
  assert( doesVertexExist( id2 ) );

  Vertices::const_iterator it = vertices.find(id1);
  return it->second.neighbors.find(id2) != it->second.neighbors.end();
}

size_t Graph::getVerticesCount() const
{
  return vertices.size();
}

size_t Graph::getDegree(int id) const
{
  assert( doesVertexExist(id) );

  Vertices::const_iterator it = vertices.find(id);
  return it->second.neighbors.size();
}

void Graph::floydWarshall(cv::Mat &distanceMatrix, int infinity) const
{
  const int edgeWeight = 1;

  const size_t n = getVerticesCount();
  distanceMatrix.create(n, n, CV_32SC1);
  distanceMatrix.setTo(infinity);
  for (Vertices::const_iterator it1 = vertices.begin(); it1 != vertices.end(); it1++)
  {
    distanceMatrix.at<int> (it1->first, it1->first) = 0;
    for (Neighbors::const_iterator it2 = it1->second.neighbors.begin(); it2 != it1->second.neighbors.end(); it2++)
    {
      assert( it1->first != *it2 );
      distanceMatrix.at<int> (it1->first, *it2) = edgeWeight;
    }
  }

  for (Vertices::const_iterator it1 = vertices.begin(); it1 != vertices.end(); it1++)
  {
    for (Vertices::const_iterator it2 = vertices.begin(); it2 != vertices.end(); it2++)
    {
      for (Vertices::const_iterator it3 = vertices.begin(); it3 != vertices.end(); it3++)
      {
        int val1 = distanceMatrix.at<int> (it2->first, it3->first);
        int val2;
        if (distanceMatrix.at<int> (it2->first, it1->first) == infinity || distanceMatrix.at<int> (it1->first,
                                                                                                   it3->first)
            == infinity)
          val2 = val1;
        else
          val2 = distanceMatrix.at<int> (it2->first, it1->first) + distanceMatrix.at<int> (it1->first, it3->first);
        distanceMatrix.at<int> (it2->first, it3->first) = std::min(val1, val2);
      }
    }
  }
}

void computeShortestPath(Mat &predecessorMatrix, int v1, int v2, vector<int> &path);
void computePredecessorMatrix(const Mat &dm, int verticesCount, Mat &predecessorMatrix);

CirclesGridFinderParameters::CirclesGridFinderParameters()
{
  minDensity = 10;
  densityNeighborhoodSize = Size2f(16, 16);
  minDistanceToAddKeypoint = 20;
  kmeansAttempts = 100;
  convexHullFactor = 1.1f;
  keypointScale = 1;

  minGraphConfidence = 9;
  vertexGain = 2;
  vertexPenalty = -5;
  edgeGain = 1;
  edgePenalty = -5;
  existingVertexGain = 0;
}

CirclesGridFinder::CirclesGridFinder(Size _patternSize, const vector<Point2f> &testKeypoints,
                                     const CirclesGridFinderParameters &_parameters) :
  patternSize(_patternSize)
{
  keypoints = testKeypoints;
  parameters = _parameters;
}

bool CirclesGridFinder::findHoles()
{
  vector<Point2f> vectors, filteredVectors, basis;
  computeEdgeVectorsOfRNG(vectors);
  filterOutliersByDensity(vectors, filteredVectors);
  vector<Graph> basisGraphs;
  findBasis(filteredVectors, basis, basisGraphs);
  findMCS(basis, basisGraphs);

  return (isDetectionCorrect());
  //CV_Error( 0, "Detection is not correct" );
}

bool CirclesGridFinder::isDetectionCorrect()
{
  if (int(holes.size()) != patternSize.height)
    return false;

  set<int> vertices;
  for (size_t i = 0; i < holes.size(); i++)
  {
    if (int(holes[i].size()) != patternSize.width)
      return false;

    for (size_t j = 0; j < holes[i].size(); j++)
    {
      vertices.insert(holes[i][j]);
    }
  }

  return int(vertices.size()) == patternSize.area();
}

void CirclesGridFinder::findMCS(const vector<Point2f> &basis, vector<Graph> &basisGraphs)
{
  Path longestPath;
  size_t bestGraphIdx = findLongestPath(basisGraphs, longestPath);
  vector<int> holesRow = longestPath.vertices;

  while (int(holesRow.size()) > std::max(patternSize.width, patternSize.height))
  {
    holesRow.pop_back();
    holesRow.erase(holesRow.begin());
  }

  if (bestGraphIdx == 0)
  {
    holes.push_back(holesRow);
    int w = holes[0].size();
    int h = holes.size();

    //parameters.minGraphConfidence = holes[0].size() * parameters.vertexGain + (holes[0].size() - 1) * parameters.edgeGain;
    //parameters.minGraphConfidence = holes[0].size() * parameters.vertexGain + (holes[0].size() / 2) * parameters.edgeGain;
    //parameters.minGraphConfidence = holes[0].size() * parameters.existingVertexGain + (holes[0].size() / 2) * parameters.edgeGain;
    parameters.minGraphConfidence = holes[0].size() * parameters.existingVertexGain;
    for (int i = h; i < patternSize.height; i++)
    {
      addHolesByGraph(basisGraphs, true, basis[1]);
    }

    //parameters.minGraphConfidence = holes.size() * parameters.existingVertexGain + (holes.size() / 2) * parameters.edgeGain;
    parameters.minGraphConfidence = holes.size() * parameters.existingVertexGain;

    for (int i = w; i < patternSize.width; i++)
    {
      addHolesByGraph(basisGraphs, false, basis[0]);
    }
  }
  else
  {
    holes.resize(holesRow.size());
    for (size_t i = 0; i < holesRow.size(); i++)
      holes[i].push_back(holesRow[i]);

    int w = holes[0].size();
    int h = holes.size();

    parameters.minGraphConfidence = holes.size() * parameters.existingVertexGain;
    for (int i = w; i < patternSize.width; i++)
    {
      addHolesByGraph(basisGraphs, false, basis[0]);
    }

    parameters.minGraphConfidence = holes[0].size() * parameters.existingVertexGain;
    for (int i = h; i < patternSize.height; i++)
    {
      addHolesByGraph(basisGraphs, true, basis[1]);
    }
  }
}

Mat CirclesGridFinder::rectifyGrid(Size detectedGridSize, const vector<Point2f>& centers,
                                   const vector<Point2f> &keypoints, vector<Point2f> &warpedKeypoints)
{
  assert( !centers.empty() );
  const float edgeLength = 30;
  const Point2f offset(150, 150);

  vector<Point2f> dstPoints;
  for (int i = 0; i < detectedGridSize.height; i++)
  {
    for (int j = 0; j < detectedGridSize.width; j++)
    {
      dstPoints.push_back(offset + Point2f(edgeLength * j, edgeLength * i));
    }
  }

  Mat H = findHomography(Mat(centers), Mat(dstPoints), CV_RANSAC);
  //Mat H = findHomography( Mat( corners ), Mat( dstPoints ) );

  vector<Point2f> srcKeypoints;
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    srcKeypoints.push_back(keypoints[i]);
  }

  Mat dstKeypointsMat;
  transform(Mat(srcKeypoints), dstKeypointsMat, H);
  vector<Point2f> dstKeypoints;
  convertPointsHomogeneous(dstKeypointsMat, dstKeypoints);

  warpedKeypoints.clear();
  for (size_t i = 0; i < dstKeypoints.size(); i++)
  {
    Point2f pt = dstKeypoints[i];
    warpedKeypoints.push_back(pt);
  }

  return H;
}

int CirclesGridFinder::findNearestKeypoint(Point2f pt) const
{
  int bestIdx = -1;
  double minDist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    double dist = norm(pt - keypoints[i]);
    if (dist < minDist)
    {
      minDist = dist;
      bestIdx = i;
    }
  }
  return bestIdx;
}

void CirclesGridFinder::addPoint(Point2f pt, vector<int> &points)
{
  int ptIdx = findNearestKeypoint(pt);
  if (norm(keypoints[ptIdx] - pt) > parameters.minDistanceToAddKeypoint)
  {
    Point2f kpt = Point2f(pt);
    keypoints.push_back(kpt);
    points.push_back(keypoints.size() - 1);
  }
  else
  {
    points.push_back(ptIdx);
  }
}

void CirclesGridFinder::findCandidateLine(vector<int> &line, int seedLineIdx, bool addRow, Point2f basisVec,
                                          vector<int> &seeds)
{
  line.clear();
  seeds.clear();

  if (addRow)
  {
    for (size_t i = 0; i < holes[seedLineIdx].size(); i++)
    {
      Point2f pt = keypoints[holes[seedLineIdx][i]] + basisVec;
      addPoint(pt, line);
      seeds.push_back(holes[seedLineIdx][i]);
    }
  }
  else
  {
    for (size_t i = 0; i < holes.size(); i++)
    {
      Point2f pt = keypoints[holes[i][seedLineIdx]] + basisVec;
      addPoint(pt, line);
      seeds.push_back(holes[i][seedLineIdx]);
    }
  }

  assert( line.size() == seeds.size() );
}

void CirclesGridFinder::findCandidateHoles(vector<int> &above, vector<int> &below, bool addRow, Point2f basisVec,
                                           vector<int> &aboveSeeds, vector<int> &belowSeeds)
{
  above.clear();
  below.clear();
  aboveSeeds.clear();
  belowSeeds.clear();

  findCandidateLine(above, 0, addRow, -basisVec, aboveSeeds);
  int lastIdx = addRow ? holes.size() - 1 : holes[0].size() - 1;
  findCandidateLine(below, lastIdx, addRow, basisVec, belowSeeds);

  assert( below.size() == above.size() );
  assert( belowSeeds.size() == aboveSeeds.size() );
  assert( below.size() == belowSeeds.size() );
}

bool CirclesGridFinder::areCentersNew(const vector<int> &newCenters, const vector<vector<int> > &holes)
{
  for (size_t i = 0; i < newCenters.size(); i++)
  {
    for (size_t j = 0; j < holes.size(); j++)
    {
      if (holes[j].end() != std::find(holes[j].begin(), holes[j].end(), newCenters[i]))
      {
        return false;
      }
    }
  }

  return true;
}

void CirclesGridFinder::insertWinner(float aboveConfidence, float belowConfidence, float minConfidence, bool addRow,
                                     const vector<int> &above, const vector<int> &below, vector<vector<int> > &holes)
{
  if (aboveConfidence < minConfidence && belowConfidence < minConfidence)
    return;

  if (addRow)
  {
    if (aboveConfidence >= belowConfidence)
    {
      if (!areCentersNew(above, holes))
        CV_Error( 0, "Centers are not new" );

      holes.insert(holes.begin(), above);
    }
    else
    {
      if (!areCentersNew(below, holes))
        CV_Error( 0, "Centers are not new" );

      holes.insert(holes.end(), below);
    }
  }
  else
  {
    if (aboveConfidence >= belowConfidence)
    {
      if (!areCentersNew(above, holes))
        CV_Error( 0, "Centers are not new" );

      for (size_t i = 0; i < holes.size(); i++)
      {
        holes[i].insert(holes[i].begin(), above[i]);
      }
    }
    else
    {
      if (!areCentersNew(below, holes))
        CV_Error( 0, "Centers are not new" );

      for (size_t i = 0; i < holes.size(); i++)
      {
        holes[i].insert(holes[i].end(), below[i]);
      }
    }
  }
}

/*
 bool CirclesGridFinder::areVerticesAdjacent(const Graph &graph, int vertex1, int vertex2)
 {
 property_map<Graph, vertex_index_t>::type index = get(vertex_index, graph);

 bool areAdjacent = false;
 graph_traits<Graph>::adjacency_iterator ai;
 graph_traits<Graph>::adjacency_iterator ai_end;

 for (tie(ai, ai_end) = adjacent_vertices(vertex1, graph); ai != ai_end; ++ai)
 {
 if (*ai == index[vertex2])
 areAdjacent = true;
 }

 return areAdjacent;
 }*/

float CirclesGridFinder::computeGraphConfidence(const vector<Graph> &basisGraphs, bool addRow,
                                                const vector<int> &points, const vector<int> &seeds)
{
  assert( points.size() == seeds.size() );
  float confidence = 0;
  const int vCount = basisGraphs[0].getVerticesCount();
  assert( basisGraphs[0].getVerticesCount() == basisGraphs[1].getVerticesCount() );

  for (size_t i = 0; i < seeds.size(); i++)
  {
    if (seeds[i] < vCount && points[i] < vCount)
    {
      if (!basisGraphs[addRow].areVerticesAdjacent(seeds[i], points[i]))
      {
        confidence += parameters.vertexPenalty;
      }
      else
      {
        confidence += parameters.vertexGain;
      }
    }

    if (points[i] < vCount)
    {
      confidence += parameters.existingVertexGain;
    }
  }

  for (size_t i = 1; i < points.size(); i++)
  {
    if (points[i - 1] < vCount && points[i] < vCount)
    {
      if (!basisGraphs[!addRow].areVerticesAdjacent(points[i - 1], points[i]))
      {
        confidence += parameters.edgePenalty;
      }
      else
      {
        confidence += parameters.edgeGain;
      }
    }
  }
  return confidence;

}

void CirclesGridFinder::addHolesByGraph(const vector<Graph> &basisGraphs, bool addRow, Point2f basisVec)
{
  vector<int> above, below, aboveSeeds, belowSeeds;
  findCandidateHoles(above, below, addRow, basisVec, aboveSeeds, belowSeeds);
  float aboveConfidence = computeGraphConfidence(basisGraphs, addRow, above, aboveSeeds);
  float belowConfidence = computeGraphConfidence(basisGraphs, addRow, below, belowSeeds);

  insertWinner(aboveConfidence, belowConfidence, parameters.minGraphConfidence, addRow, above, below, holes);
}

void CirclesGridFinder::filterOutliersByDensity(const vector<Point2f> &samples, vector<Point2f> &filteredSamples)
{
  if (samples.empty())
    CV_Error( 0, "samples is empty" );

  filteredSamples.clear();

  for (size_t i = 0; i < samples.size(); i++)
  {
    Rect_<float> rect(samples[i] - Point2f(parameters.densityNeighborhoodSize) * 0.5,
                      parameters.densityNeighborhoodSize);
    int neighborsCount = 0;
    for (size_t j = 0; j < samples.size(); j++)
    {
      if (rect.contains(samples[j]))
        neighborsCount++;
    }
    if (neighborsCount >= parameters.minDensity)
      filteredSamples.push_back(samples[i]);
  }

  if (filteredSamples.empty())
    CV_Error( 0, "filteredSamples is empty" );
}

void CirclesGridFinder::findBasis(const vector<Point2f> &samples, vector<Point2f> &basis, vector<Graph> &basisGraphs)
{
  basis.clear();
  Mat bestLabels;
  TermCriteria termCriteria;
  Mat centers;
  int clustersCount = 4;
  kmeans(Mat(samples).reshape(1, 0), clustersCount, bestLabels, termCriteria, parameters.kmeansAttempts,
         KMEANS_RANDOM_CENTERS, &centers);
  assert( centers.type() == CV_32FC1 );

  vector<int> basisIndices;
  //TODO: only remove duplicate
  for (int i = 0; i < clustersCount; i++)
  {
    int maxIdx = (fabs(centers.at<float> (i, 0)) < fabs(centers.at<float> (i, 1)));
    if (centers.at<float> (i, maxIdx) > 0)
    {
      Point2f vec(centers.at<float> (i, 0), centers.at<float> (i, 1));
      basis.push_back(vec);
      basisIndices.push_back(i);
    }
  }
  if (basis.size() != 2)
    CV_Error( 0, "Basis size is not 2");

  if (basis[1].x > basis[0].x)
  {
    std::swap(basis[0], basis[1]);
    std::swap(basisIndices[0], basisIndices[1]);
  }

  const float minBasisDif = 2;
  if (norm(basis[0] - basis[1]) < minBasisDif)
    CV_Error( 0, "degenerate basis" );

  vector<vector<Point2f> > clusters(2), hulls(2);
  for (size_t k = 0; k < samples.size(); k++)
  {
    int label = bestLabels.at<int> (k, 0);
    int idx = -1;
    if (label == basisIndices[0])
      idx = 0;
    if (label == basisIndices[1])
      idx = 1;
    if (idx >= 0)
    {
      clusters[idx].push_back(basis[idx] + parameters.convexHullFactor * (samples[k] - basis[idx]));
    }
  }
  for (size_t i = 0; i < basis.size(); i++)
  {
    convexHull(Mat(clusters[i]), hulls[i]);
  }

  basisGraphs.resize(basis.size(), Graph(keypoints.size()));
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    for (size_t j = 0; j < keypoints.size(); j++)
    {
      if (i == j)
        continue;

      Point2f vec = keypoints[i] - keypoints[j];

      for (size_t k = 0; k < hulls.size(); k++)
      {
        if (pointPolygonTest(Mat(hulls[k]), vec, false) >= 0)
        {
          basisGraphs[k].addEdge(i, j);
        }
      }
    }
  }
}

void CirclesGridFinder::computeEdgeVectorsOfRNG(vector<Point2f> &vectors, Mat *drawImage) const
{
  vectors.clear();

  //TODO: use more fast algorithm instead of naive N^3
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    for (size_t j = 0; j < keypoints.size(); j++)
    {
      if (i == j)
        continue;

      Point2f vec = keypoints[i] - keypoints[j];
      double dist = norm(vec);

      bool isNeighbors = true;
      for (size_t k = 0; k < keypoints.size(); k++)
      {
        if (k == i || k == j)
          continue;

        double dist1 = norm(keypoints[i] - keypoints[k]);
        double dist2 = norm(keypoints[j] - keypoints[k]);
        if (dist1 < dist && dist2 < dist)
        {
          isNeighbors = false;
          break;
        }
      }

      if (isNeighbors)
      {
        vectors.push_back(keypoints[i] - keypoints[j]);
        if (drawImage != 0)
        {
          line(*drawImage, keypoints[i], keypoints[j], Scalar(255, 0, 0), 2);
          circle(*drawImage, keypoints[i], 3, Scalar(0, 0, 255), -1);
          circle(*drawImage, keypoints[j], 3, Scalar(0, 0, 255), -1);
        }
      }
    }
  }
}

void computePredecessorMatrix(const Mat &dm, int verticesCount, Mat &predecessorMatrix)
{
  assert( dm.type() == CV_32SC1 );
  predecessorMatrix.create(verticesCount, verticesCount, CV_32SC1);
  predecessorMatrix = -1;
  for (int i = 0; i < predecessorMatrix.rows; i++)
  {
    for (int j = 0; j < predecessorMatrix.cols; j++)
    {
      int dist = dm.at<int> (i, j);
      for (int k = 0; k < verticesCount; k++)
      {
        if (dm.at<int> (i, k) == dist - 1 && dm.at<int> (k, j) == 1)
        {
          predecessorMatrix.at<int> (i, j) = k;
          break;
        }
      }
    }
  }
}

void computeShortestPath(Mat &predecessorMatrix, int v1, int v2, vector<int> &path)
{
  if (predecessorMatrix.at<int> (v1, v2) < 0)
  {
    path.push_back(v1);
    return;
  }

  computeShortestPath(predecessorMatrix, v1, predecessorMatrix.at<int> (v1, v2), path);
  path.push_back(v2);
}

size_t CirclesGridFinder::findLongestPath(vector<Graph> &basisGraphs, Path &bestPath)
{
  vector<Path> longestPaths(1);
  vector<int> confidences;

  size_t bestGraphIdx = 0;
  const int infinity = -1;
  for (size_t graphIdx = 0; graphIdx < basisGraphs.size(); graphIdx++)
  {
    const Graph &g = basisGraphs[graphIdx];
    Mat distanceMatrix;
    g.floydWarshall(distanceMatrix, infinity);
    Mat predecessorMatrix;
    computePredecessorMatrix(distanceMatrix, g.getVerticesCount(), predecessorMatrix);

    double maxVal;
    Point maxLoc;
    assert (infinity < 0);
    minMaxLoc(distanceMatrix, 0, &maxVal, 0, &maxLoc);

    if (maxVal > longestPaths[0].length)
    {
      longestPaths.clear();
      confidences.clear();
      bestGraphIdx = graphIdx;
    }
    if (longestPaths.empty() || (maxVal == longestPaths[0].length && graphIdx == bestGraphIdx))
    {
      Path path = Path(maxLoc.x, maxLoc.y, cvRound(maxVal));
      computeShortestPath(predecessorMatrix, maxLoc.x, maxLoc.y, path.vertices);
      longestPaths.push_back(path);

      int conf = 0;
      for (size_t v2 = 0; v2 < path.vertices.size(); v2++)
      {
        conf += basisGraphs[1 - (int)graphIdx].getDegree(v2);
      }
      confidences.push_back(conf);
    }
  }
  //if( bestGraphIdx != 0 )
  //CV_Error( 0, "" );

  int maxConf = -1;
  int bestPathIdx = -1;
  for (size_t i = 0; i < confidences.size(); i++)
  {
    if (confidences[i] > maxConf)
    {
      maxConf = confidences[i];
      bestPathIdx = i;
    }
  }

  //int bestPathIdx = rand() % longestPaths.size();
  bestPath = longestPaths.at(bestPathIdx);
  bool needReverse = (bestGraphIdx == 0 && keypoints[bestPath.lastVertex].x < keypoints[bestPath.firstVertex].x)
      || (bestGraphIdx == 1 && keypoints[bestPath.lastVertex].y < keypoints[bestPath.firstVertex].y);
  if (needReverse)
  {
    std::swap(bestPath.lastVertex, bestPath.firstVertex);
    std::reverse(bestPath.vertices.begin(), bestPath.vertices.end());
  }
  return bestGraphIdx;
}

void CirclesGridFinder::drawBasis(const vector<Point2f> &basis, Point2f origin, Mat &drawImg) const
{
  for (size_t i = 0; i < basis.size(); i++)
  {
    Point2f pt(basis[i]);
    line(drawImg, origin, origin + pt, Scalar(0, i * 255, 0), 2);
  }
}

void CirclesGridFinder::drawBasisGraphs(const vector<Graph> &basisGraphs, Mat &drawImage, bool drawEdges,
                                        bool drawVertices) const
{
  //const int vertexRadius = 1;
  const int vertexRadius = 3;
  const Scalar vertexColor = Scalar(0, 0, 255);
  const int vertexThickness = -1;

  const Scalar edgeColor = Scalar(255, 0, 0);
  //const int edgeThickness = 1;
  const int edgeThickness = 2;

  if (drawEdges)
  {
    for (size_t i = 0; i < basisGraphs.size(); i++)
    {
      for (size_t v1 = 0; v1 < basisGraphs[i].getVerticesCount(); v1++)
      {
        for (size_t v2 = 0; v2 < basisGraphs[i].getVerticesCount(); v2++)
        {
          if (basisGraphs[i].areVerticesAdjacent(v1, v2))
          {
            line(drawImage, keypoints[v1], keypoints[v2], edgeColor, edgeThickness);
          }
        }
      }
    }
  }
  if (drawVertices)
  {
    for (size_t v = 0; v < basisGraphs[0].getVerticesCount(); v++)
    {
      circle(drawImage, keypoints[v], vertexRadius, vertexColor, vertexThickness);
    }
  }
}

void CirclesGridFinder::drawHoles(const Mat &srcImage, Mat &drawImage) const
{
  //const int holeRadius = 4;
  //const int holeRadius = 2;
  //const int holeThickness = 1;
  const int holeRadius = 3;
  const int holeThickness = -1;

  //const Scalar holeColor = Scalar(0, 0, 255);
  const Scalar holeColor = Scalar(0, 255, 0);

  if (srcImage.channels() == 1)
    cvtColor(srcImage, drawImage, CV_GRAY2RGB);
  else
    srcImage.copyTo(drawImage);

  for (size_t i = 0; i < holes.size(); i++)
  {
    for (size_t j = 0; j < holes[i].size(); j++)
    {
      if (j != holes[i].size() - 1)
        line(drawImage, keypoints[holes[i][j]], keypoints[holes[i][j + 1]], Scalar(255, 0, 0), 2);
      if (i != holes.size() - 1)
        line(drawImage, keypoints[holes[i][j]], keypoints[holes[i + 1][j]], Scalar(255, 0, 0), 2);

      //circle(drawImage, keypoints[holes[i][j]], holeRadius, holeColor, holeThickness);
      circle(drawImage, keypoints[holes[i][j]], holeRadius, holeColor, holeThickness);
    }
  }
}

Size CirclesGridFinder::getDetectedGridSize() const
{
  if (holes.size() == 0)
    return Size(0, 0);

  return Size(holes[0].size(), holes.size());
}

void CirclesGridFinder::getHoles(vector<Point2f> &outHoles) const
{
  outHoles.clear();

  for (size_t i = 0; i < holes.size(); i++)
  {
    for (size_t j = 0; j < holes[i].size(); j++)
    {
      outHoles.push_back(keypoints[holes[i][j]]);
    }
  }
}
}
