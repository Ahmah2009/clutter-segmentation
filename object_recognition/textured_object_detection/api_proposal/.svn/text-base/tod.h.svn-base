/* tod.h
 basic tod interface, no outside dependencies besides opencv
 */

#include <opencv2/core/core.hpp>

namespace tod
{
class Serializable
{
public:
  virtual ~Serializable()
  {
  }
  /** Opencv serialization
   */
  virtual void write(const cv:FileStorage& fs) const = 0;
  /**
   */
  virtual void read(const cv::FileNode& fn) = 0;

  /** Convenience serialization
   */
  virtual void write(const std::string& yml_file) const = 0;
  /** Convenience deserialization
   */
  virtual void read(const std::string& yml_file) = 0;
};

/** Drawable interface, this object will draw stuff to a cv::Mat, draw whatever...
 */
class Drawable
{
  virtual ~Drawable()
  {
  }
  /** draw something to a matrix...
   */
  void draw(cv::Mat& out, int flags = 0) const = 0;
};

/** Structure holding all relevant camera calibration parameters
 *
 */
class Camera : public Serializable
{
public:
  Camera();
  /** Deserialize from a camera.yml file - opencv style yaml calibration
   */
  Camera(const std::string& camera_yml);
  /** Deserialize from an opencv fs object
   */
  Camera(const cv::FileNode& fn);

  virtual ~Camera();

  cv::Mat K; //!< intrinsics
  cv::Mat D; //!< distortion coeffs
  cv::Mat Kinv; //!< inverse intrinsics - precomputed for projecting points
  cv::Size image_size; //!< image size - the size of the image that this calibration is based on
};

/** A pose data structure, contains the rotation and translation of an object
 */
struct PoseRT
{
  cv::Mat rvec; //!<rodriguez formula rotation 3 vector
  cv::Mat tvec; //!<3 vector, translation
};

/** Encapsulates all data that is relevant to features2d, keypoints, descriptors, masks, images
 */
class Features2d : public Serializable, Drawable
{
public:
  Features2d();
  Features2d(const cv::Mat& gray);
  virtual ~Features2d();

  cv::Mat gray;
  cv::Mat mask;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  PoseRT pose;
  std::string image_name;
};

class Features3d : public Serializable
{
public:
  typedef vector<Point3f> Cloud; //!< Point Cloud , dense or not
  typedef vector<pair<int, int> > Mapping; //!< Mapping (features,cloud) - first is the feature index, second is the cloud index

  Features3d();
  //defaults to one to one mapping
  Features3d(const Camera& camera, const Features2d& features, const Cloud& cloud, const Mapping& mapping = Mapping());

  virtual ~Features3d();

  /** get the camera object, that projects the cloud into the frame that the features are in
   */
  const Camera& camera() const;

  /* \brief get access to the underlying Features2d.
   * However it should be const as to not mess up the mapping between cloud and features.
   */
  const Features2d& features() const;
  /* \brief get access to the underlying Cloud.
   * However it should be const as to not mess up the mapping between cloud and features.
   */
  const Cloud& cloud() const;

  /**\brief get the Mapping between features
   */
  const Mapping& mapping() const;

private:

  Camera camera_;
  Features2d features_;
  Cloud cloud_;
  std::vector<std::pair<int, int> > mapping_;
};

/** The basic textured object data structure.  This encapsulates the idea of an object with many observations - or view based.
 * TODO make it also modelbased - if there is good justification
 * Think, Odwalla bottle, or Egg Poacher
 */
struct Object
{
  std::vector<cv::Mat> getDescriptors() const; //get a stacked vector of all the descriptors from each observation (Features3d)
  vector<Features3d> observations; //!< view based representation of the object
  int id; //!< An id for the object, useful for database referals
  std::string name; //!< Human readable name for the TexturedObject detector

  //TODO for one to many matching - need to calculate the offset of descriptors in the observation set
};

/** The database style object representation. This is used by the Detector,  Its only job is to store and retrieve tod::Object instances
 */
class TrainingBase
{
public:
  typedef std::vector<cv::Ptr<Object> > ObjectCollection;
  TrainingBase(const std::string& path);
  TrainingBase(const ObjectCollection& objects);

  /**Get the number of objects
   */
  size_t size() const;
  const cv::Ptr<Object> getObject(int objectID) const;

protected:
  ObjectCollection objects_;
};

class ObjectPose
{
private:
  const Ptr<Object> object_;

public:
  ObjectPose(const Ptr<Object>& object);
  ObjectPose();
  ObjectPose(const ObjectPose& objectPose);

  PoseRT pose;
  const Ptr<Object> getObject() const;

  // add more fields as it is more clear how confidence information could be used outside
};

class TODetector
{
  class Params
  {
    void read(const string& path);
    void write(const string& path);
  };

  TODetector(const Ptr<TrainingBase>& trainingBase, const Params& params);
  TODetector(const Ptr<TrainingBase>& trainingBase, const string& params);

  void detect(const Features3d& features, vector<ObjectPose>& objectPoses);
};
}
