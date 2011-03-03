/*
 * Recognizer.h
 *
 *  Created on: Feb 3, 2011
 *      Author: Alexander Shishkov
 */

#ifndef RECOGNIZER_H_
#define RECOGNIZER_H_

#include <opencv2/core/core.hpp>
#include <tod/core/TrainingBase.h>
#include <tod/core/Features2d.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/detecting/Matcher.h>

namespace tod
{
  struct ObjectInfo
  {
    int objectId;
    std::string objectName;
    cv::Mat rvec;
    cv::Mat tvec;
    int imgIdx;

    ObjectInfo();
    ObjectInfo(const Guess& guess);
    ObjectInfo & operator =(const ObjectInfo& object);
    ObjectInfo(const ObjectInfo& object);
  };

  class Recognizer
  {
  public:
    Recognizer();
    Recognizer(TrainingBase* base, cv::Ptr<Matcher> matcher, GuessGeneratorParameters* params, int verbose);
    virtual ~Recognizer();
    virtual void match(Features2d& test, std::vector<ObjectInfo>& objects) = 0;

  protected:
    TrainingBase* base;
    cv::Ptr<Matcher> matcher;
    GuessGeneratorParameters* params;
    int verbose;
  };

  class KinectRecognizer: public Recognizer
  {
  public:
    //TODO: fix matcher
    KinectRecognizer(TrainingBase* base, cv::Ptr<Matcher> matcher,
                     GuessGeneratorParameters* params, int verbose, std::string baseDirectory);
    ~KinectRecognizer();
    virtual void match(Features2d& test2d, std::vector<ObjectInfo>& objects);
  private:
    std::string baseDirectory;

    void sortObjectMatchesByView(vector<cv::DMatch> &objectMatches, std::map<int, std::vector<cv::DMatch> > & viewMatches,
                                 std::vector<std::pair<int, int > > & viewSizes);
  };

  class TODRecognizer: public Recognizer
  {
  public:
    TODRecognizer(TrainingBase* base, cv::Ptr<Matcher> matcher,
                     GuessGeneratorParameters* params, int verbose,
                     std::string baseDirectory, float maxDistance);
    ~TODRecognizer();
    virtual void match(Features2d& test2d, std::vector<ObjectInfo>& objects);
  private:
    std::string baseDirectory;
    float maxDistance;
    void printMatches(std::vector<int>& objectIds);
    void drawProjections(cv::Mat& image, int id, const std::vector<Guess>& guesses, const std::string& baseDirectory);
    void drawClusters(const cv::Mat img, const std::vector<cv::Point2f>& imagePoints,
                      const std::vector<std::vector<int> >& indices);
  };
}

#endif /* RECOGNIZER_H_ */
