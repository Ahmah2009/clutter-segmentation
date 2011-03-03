/*
 * PoseEstimate.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#ifndef POSEESTIMATE_H_
#define POSEESTIMATE_H_

#include <tod/core/PoseRT.h>
#include <tod/core/TexturedObject.h>

namespace tod
{

class PoseEstimate
{
public:
  PoseEstimate();
  virtual ~PoseEstimate();

  PoseEstimate(const cv::Ptr<TexturedObject>& object, const PoseRT& rt);

  PoseEstimate(const PoseEstimate& objectPose);

  const PoseRT& pose() const;

  const cv::Ptr<TexturedObject> getObject() const;

private:
  const cv::Ptr<TexturedObject> object_;
  PoseRT pose_;
  // add more fields as it is more clear how confidence information could be used outside
};

}

#endif /* POSEESTIMATE_H_ */
