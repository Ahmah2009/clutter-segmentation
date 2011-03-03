/*
 * pose.h
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

#ifndef TOD_POSE_H_
#define TOD_POSE_H_
#include <tod/core/common.h>
namespace tod
{
/**\brief A pose data structure, contains the rotation and translation of an object
 */
class PoseRT : public Serializable
{
public:
  PoseRT();
  PoseRT(const cv::Mat rvec, const cv::Mat& tvec);
  virtual ~PoseRT()
  {
  }
  cv::Mat rvec; //!<rodriguez formula rotation 3 vector
  cv::Mat tvec; //!<3 vector, translation
  bool estimated;

  virtual void write(cv::FileStorage& fs) const;
  virtual void read(const cv::FileNode& fn);

  static const std::string YAML_NODE_NAME;
  cv::Vec4f toPlanarCoefficients() const;
  //PoseRT relative(const PoseRT& rhs) const;
  //PoseRT inverse() const;

  void enforceForm();

};

}
inline std::ostream& operator << (std::ostream& out, const tod::PoseRT& pose){
  out << "r="<<pose.rvec << ", t=" << pose.tvec;
  return out;
}
#endif /* POSE_H_ */
