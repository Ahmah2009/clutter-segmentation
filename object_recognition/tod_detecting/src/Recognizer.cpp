/*
 * Recognizer.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: Alexander Shishkov
 */

#include <tod/detecting/Recognizer.h>
#include <tod/detecting/Cluster.h>
#include <tod/detecting/Filter.h>
#include <tod/detecting/GuessQualifier.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
typedef std::pair<int, int > idx_pair_t;

using namespace tod;
using namespace cv;
using namespace std;

ObjectInfo::ObjectInfo()
{
  imgIdx = 0;
  objectId = 0;
  objectName = "";
  rvec.create(3, 1, CV_64FC1);
  rvec = cv::Scalar(0.0);
  tvec.create(3, 1, CV_64FC1);
  tvec = cv::Scalar(0.0);
}

ObjectInfo::ObjectInfo(const Guess& guess)
{
  objectId = guess.getObject()->id;
  objectName = guess.getObject()->name;
  imgIdx = guess.imageIndex;
  guess.pose().rvec.copyTo(rvec);
  guess.pose().tvec.copyTo(tvec);
}

ObjectInfo & ObjectInfo::operator =(const ObjectInfo& object)
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


ObjectInfo::ObjectInfo(const ObjectInfo& object)
{
  objectId = object.objectId;
  objectName = object.objectName;
  object.rvec.copyTo(rvec);
  object.tvec.copyTo(tvec);
  imgIdx = object.imgIdx;
}

Recognizer::Recognizer()
{
  base = NULL;
  params = NULL;
  matcher = NULL;
  verbose = 0;
}

Recognizer::Recognizer(TrainingBase* base_, cv::Ptr<Matcher> matcher_,
                       GuessGeneratorParameters* params_, int verbose_)
{
  base = base_;
  matcher = matcher_;
  params = params_;
  verbose = verbose_;
}

Recognizer::~Recognizer()
{
  base = NULL;
  params = NULL;
  matcher = NULL;
}

KinectRecognizer::KinectRecognizer(TrainingBase* base, cv::Ptr<Matcher> matcher,
                                   GuessGeneratorParameters* params, int verbose, string baseDirectory_):
                                   Recognizer(base, matcher, params, verbose), baseDirectory(baseDirectory_)
{
}

KinectRecognizer::~KinectRecognizer()
{
}

bool compareGreaterDMatch(const DMatch & lhs, const DMatch & rhs)
{
  return lhs.imgIdx > rhs.imgIdx; //  && lhs.imgIdx == rhs.imgIdx ? lhs.distance < rhs.distance : true;
}

void KinectRecognizer::sortObjectMatchesByView(vector<DMatch> &objectMatches, std::map<int, std::vector<DMatch> > & viewMatches,
                                                    std::vector<std::pair<int, int > > & viewSizes)
{
 viewMatches.clear();
 viewSizes.clear();
 if (objectMatches.empty())
   return;

 std::sort(objectMatches.begin(), objectMatches.end(), compareGreaterDMatch);
 std::map<int, int > viewsizes_;
 for (size_t i = 0; i < objectMatches.size(); i++)
 {
   viewMatches[objectMatches[i].imgIdx].push_back(objectMatches[i]);
   viewsizes_[objectMatches[i].imgIdx]++;
 }
 foreach (const idx_pair_t & x, viewsizes_)
 {
   viewSizes.push_back(x);
 }
 std::sort(viewSizes.begin(), viewSizes.end(), Matcher::pair_second_greater<idx_pair_t>);
}

void KinectRecognizer::match(Features2d& test2d, std::vector<ObjectInfo>& objects)
{
  objects.clear();

  Features3d test(test2d);
  matcher->match(test.features().descriptors);

  if (verbose == 2)
    Matcher::drawMatches(*base, matcher, test.features().image, test.features().keypoints, baseDirectory);

  std::vector<std::pair<int, int > > labels_sizes;
  matcher->getLabelSizes(labels_sizes);
  vector<int > objectIds;
  base->getObjectIds(objectIds);
  vector<DMatch> objectMatches;

  cv::Mat projection;
  vector<Guess> guesses_all;
  foreach (const idx_pair_t & x, labels_sizes)
  {
    if (x.second < params->minInliersCount)
      break;
    matcher->getObjectMatches(x.first, objectMatches);
    std::map<int, std::vector<DMatch> > viewMatches;
    std::vector<std::pair<int, int > > viewSizes;
    sortObjectMatchesByView(objectMatches, viewMatches, viewSizes);
    GuessGenerator generator(*params);
    vector<Guess> guesses;

    for(size_t i = 0; i < viewSizes.size() /* && i < 3 TODO parameterize this*/; i++)
    {
      generator.calculateGuesses(base->getObject(x.first), viewMatches[viewSizes[i].first],
          test.features().keypoints, test.features().image, guesses);
      guesses_all.insert(guesses_all.end(), guesses.begin(), guesses.end());
    }
  }
  if (guesses_all.size())
  {
    cv::Mat correspondence;
    if (verbose)
    {
      namedWindow("correspondence", CV_WINDOW_KEEPRATIO);
    }

    foreach (const Guess & guess, guesses_all)
    {
      objects.push_back(ObjectInfo(guess));
      if (verbose)
      {
        guess.draw(projection, 0, ".");
        guess.draw(correspondence, 1, baseDirectory);
        if (!correspondence.empty())
        {
          imshow("correspondence", correspondence);
          waitKey(0);
        }
      }
    }
  }
  if (!projection.empty() && verbose)
  {
    namedWindow("projection", CV_WINDOW_KEEPRATIO);
    imshow("projection", projection);
    waitKey(0);
  }
}


TODRecognizer::TODRecognizer(TrainingBase* base, cv::Ptr<Matcher> matcher,
                                   GuessGeneratorParameters* params, int verbose,
                                   string baseDirectory_, float maxDistance_):
                                   Recognizer(base, matcher, params, verbose),
                                   baseDirectory(baseDirectory_), maxDistance(maxDistance_)
{
}

TODRecognizer::~TODRecognizer()
{
}

void TODRecognizer::printMatches(vector<int>& objectIds)
{
  int matchesSize = 0;
  foreach(int id, objectIds)
  {
    vector<DMatch> objectMatches;
    matcher->getObjectMatches(id, objectMatches);
    matchesSize += objectMatches.size();
  }

  cout << "Total matches: " << matchesSize << endl;

  int maxMatch = 0, maxMatchObj = -1;
  cout << "Matches for each object (row) per view (column)" << endl;
  foreach(int id, objectIds)
  {
    cout << "Object " << id << ":" << "\t";
    const Ptr<TexturedObject>& object = base->getObject(id);
    for (size_t obsInd = 0; obsInd < object->observations.size(); obsInd++)
    {
      vector<DMatch> obsMatches;
      matcher->getImageMatches(id, obsInd, obsMatches);
      cout << obsMatches.size() << "\t";
      if (maxMatch < (int)obsMatches.size())
      {
        maxMatch = obsMatches.size();
        maxMatchObj = id;
      }
    }
    cout << endl;
  }
  cout << "Max match is " << maxMatch << " for object " << maxMatchObj << endl;
}

void TODRecognizer::drawProjections(Mat& image, int id, const vector<Guess>& guesses, const string& baseDirectory)
{
  if (guesses.empty())
    return;

  foreach(const Guess& guess, guesses)
  {
    guess.draw(image, 0, ".");
  }
}

void TODRecognizer::match(Features2d& test, std::vector<ObjectInfo>& objects)
{
  objects.clear();

  matcher->match(test.descriptors);
  if (verbose == 2)
    Matcher::drawMatches(*base, matcher, test.image, test.keypoints, baseDirectory);

  vector<int> objectIds;
  base->getObjectIds(objectIds);

  printMatches(objectIds);

  Mat drawImage;
  if (verbose)
  {
    if (test.image.channels() > 1)
      test.image.copyTo(drawImage);
    else
      cvtColor(test.image, drawImage, CV_GRAY2BGR);
  }

  foreach(int id, objectIds)
  {
    vector<DMatch> objectMatches;
    matcher->getObjectMatches(id, objectMatches);

    vector<vector<int> > clusterIndices;
    vector<int> imgIndices;
    vector<Point2f> points;
    for (size_t k = 0; k < objectMatches.size(); k++)
    {
      points.push_back(test.keypoints[objectMatches[k].queryIdx].pt);
      imgIndices.push_back((int)objectMatches[k].imgIdx);
    }
    ClusterBuilder clusterBuilder(maxDistance);
    clusterBuilder.clusterPoints(points, imgIndices, clusterIndices);
    if (verbose == 2)
      drawClusters(test.image.clone(), points, clusterIndices);

    GuessGenerator generator(*params);
    vector<Guess> guesses;
    const Ptr<TexturedObject>& object = base->getObject(id);
    generator.calculateGuesses(object, clusterIndices, objectMatches, test.keypoints, test.image, guesses);

    GuessQualifier qualifier(object, objectMatches, test.keypoints, *params);
    qualifier.clarify(guesses);
    StdDevFilter stdFilter(object, *params);
    stdFilter.filterGuesses(guesses);
    OverlappingFilter overlappingFilter(object, test.image.size());
    overlappingFilter.filterGuesses(guesses);

    foreach(const Guess& guess, guesses)
    {
      objects.push_back(ObjectInfo(guess));
    }
    if (verbose)
      drawProjections(drawImage, id, guesses, baseDirectory);
  }

  if (verbose)
  {
    namedWindow("projection", CV_WINDOW_KEEPRATIO);
    imshow("projection", drawImage);
    waitKey(0);
  }
}

void TODRecognizer::drawClusters(const cv::Mat img, const std::vector<cv::Point2f>& imagePoints,
                               const vector<vector<int> >& indices)
{
  string clusterWinName = "clusters";
  namedWindow(clusterWinName, CV_WINDOW_AUTOSIZE);
  //cout << "NumClusters = " << indices.size() << endl;
  vector<Scalar> colors(indices.size());
  for (size_t k = 0; k < indices.size(); k++)
    colors[k] = CV_RGB(rand() % 255, rand() % 255, rand() % 255);
  Size size(img.cols, img.rows);
  Mat drawImg(size, CV_MAKETYPE(img.depth(), 3));
  drawImg.setTo(Scalar::all(0));
  Mat drawImg1 = drawImg(Rect(0, 0, img.cols, img.rows));
  cvtColor(img, drawImg1, CV_GRAY2RGB);

  for (size_t j = 0; j < indices.size(); j++)
  {
    for (size_t k = 0; k < indices[j].size(); k++)
      circle(drawImg, Point(imagePoints[indices[j][k]].x, imagePoints[indices[j][k]].y), 7, colors[j], 2);
  }

  Mat smallDrawImg;
  resize(drawImg, smallDrawImg, Size(), 0.3, 0.3);
  imshow(clusterWinName, smallDrawImg);
  waitKey(0);
}
