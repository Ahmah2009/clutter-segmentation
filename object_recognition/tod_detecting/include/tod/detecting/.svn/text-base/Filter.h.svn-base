/*
 * Filter.h
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <tod/detecting/GuessGenerator.h>
#include <tod/core/TexturedObject.h>
#include <opencv2/core/core.hpp>

namespace tod
{
  class Filter
  {
  public:
    Filter(const cv::Ptr<TexturedObject>& texturedObject);
    virtual ~Filter() {};
    virtual void filterGuesses(std::vector<Guess>& guesses) = 0;
  protected:
    const cv::Ptr<TexturedObject>& object;
  };

  class StdDevFilter: public Filter
  {
  public:
    StdDevFilter(const cv::Ptr<TexturedObject>& texturedObject, const GuessGeneratorParameters& generatorParams);
    ~StdDevFilter();
    void filterGuesses(std::vector<Guess>& guesses);
  private:
    const GuessGeneratorParameters& params;
  };

  class OverlappingFilter: public Filter
  {
  public:
    OverlappingFilter(const cv::Ptr<TexturedObject>& texturedObject, const cv::Size& size);
    ~OverlappingFilter();
    void filterGuesses(std::vector<Guess>& guesses);
  private:
    cv::Size size;
    void initProjectedPoints(std::vector<Guess>& guesses);
  };
}
#endif /* FILTER_H_ */


