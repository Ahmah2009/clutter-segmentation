#include <highgui.h>
#include <fstream>
#include <iomanip>

#include <visualization_msgs/Marker.h>

#include "posest/pnp_ransac.h"
#include "textured_object_detection/training_set.h"
#include "textured_object_detection/shared_functions.h"

#define RANSAC_ITERATIONS 200

using namespace cv;
using namespace std;

//old format
void ObjectInfo::write(FileStorage& fs) const
{
  fs << objectName << "{" << "objectId" << objectId << "rvec" << rvec << "tvec" << tvec << "}";
}

void ObjectInfo::saveObjects(FileStorage& fs, const vector<ObjectInfo>& objects)
{
  for (size_t i = 0; i < objects.size(); i++)
  {
    objects[i].write(fs);
  }
}

//new format
void ObjectInfo::write(FileStorage& fs, int image_index, int object_index) const
{
  stringstream out;
  out << object_index;

  fs << "object" + out.str() << "{" << "id" << objectId << "name" << objectName << "rvec" << rvec << "tvec" << tvec
      << "imageIndex" << image_index << "}";
}

void ObjectInfo::saveObjects(FileStorage& fs, int image_index, const vector<ObjectInfo>& objects,
                             int& totalObjectsCount)
{
  for (size_t i = 0; i < objects.size(); i++)
  {
    objects[i].write(fs, image_index, totalObjectsCount + 1);
    totalObjectsCount++;
  }
}

void ObjectInfo::loadObjects(cv::FileNode& objectsNode, std::vector<int>& image_indexes,
                             std::vector<ObjectInfo>& objects)
{
  image_indexes.clear();
  objects.clear();

  int objectsCount = 0;
  objectsNode["objectsCount"] >> objectsCount;

  objects.resize(objectsCount);
  image_indexes.resize(objectsCount);

  for (int i = 0; i < objectsCount; i++)
  {
    stringstream out;
    out << i + 1;
    FileNode object = objectsNode["object" + out.str()];
    object["id"] >> objects[i].objectId;
    object["name"] >> objects[i].objectName;
    object["rvec"] >> objects[i].rvec;
    object["tvec"] >> objects[i].tvec;
    if (!object["idx"].empty())
      object["idx"] >> objects[i].imgIdx;
    object["imageIndex"] >> image_indexes[i];
  }
}

struct Cluster
{
  Point2f center;
  int imgInd;
  vector<int> pIndices;
  float sumx;
  float sumy;

  Cluster(Point2f _center, int index, int _imgInd)
  {
    center = _center;
    imgInd = _imgInd;
    pIndices.push_back(index);
    sumx = _center.x;
    sumy = _center.y;
  }
};

TrainingSet::~TrainingSet()
{
  for (size_t i = 0; i < objects.size(); i++)
    delete objects[i];
  delete flann_index;
}

//default we use FAST detector
FeatureDetector* TrainingSet::createDetector(const string& detectorType)
{
  printf("Created a %s detector with threshold %f\n", detectorType.c_str(), params.detectorThreshold);
  FeatureDetector* fd = 0;
  if (!detectorType.compare("FAST"))
  {
    fd
        = new FastFeatureDetector((int)params.detectorThreshold/*threshold*/, params.nonmaxSuppression/*nonmax_suppression*/);
  }
  else if (!detectorType.compare("MFAST"))
  {
    Ptr<FeatureDetector> detector = new FastFeatureDetector((int)params.detectorThreshold/*threshold*/,
                                                            params.nonmaxSuppression/*nonmax_suppression*/);
    fd = new PyramidAdaptedFeatureDetector(detector, 5);
  }
  else if (!detectorType.compare("STAR"))
  {
    fd
        = new StarFeatureDetector(16/*max_size*/, (int)params.detectorThreshold/*response_threshold*/,
                                  10/*line_threshold_projected*/, 8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/);
  }
  else if (!detectorType.compare("SIFT"))
  {
    fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  }
  else if (!detectorType.compare("SURF"))
  {
    fd = new SurfFeatureDetector(params.detectorThreshold/*hessian_threshold*/, 5/*octaves*/, 4/*octave_layers*/);
  }
  else if (!detectorType.compare("MSER"))
  {
    fd = new MserFeatureDetector(5/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.25f/*max_variation*/,
                                 0.2/*min_diversity*/, 200/*max_evolution*/,
                                 params.detectorThreshold/*area_threshold*/, 0.003/*min_margin*/, 5/*edge_blur_size*/);
  }
  else if (!detectorType.compare("GFTT"))
  {
    fd = new GoodFeaturesToTrackDetector(1000/*maxCorners*/, params.detectorThreshold/*qualityLevel*/,
                                         1./*minDistance*/, 3/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/);
  }
#ifdef PAS_FEATURES
    else if (!detectorType.compare("PAS"))
    {
      fd = new PASDetector;
    }
#endif
else    assert(0);
    return fd;
  }

  //default we use SURF descriptor
DescriptorExtractor* TrainingSet::createDescriptorExtractor(const string& descriptorType)
{
  DescriptorExtractor* de = 0;
  if (!descriptorType.compare("SIFT"))
  {
    de = new SiftDescriptorExtractor;
  }
  else if (!descriptorType.compare("SURF"))
  {
    de = new SurfDescriptorExtractor(3/*nOctaves*/, 4/*nOctaveLayers*/, false/*extended*/);
  }
#ifdef PAS_FEATURES
    else if (!descriptorType.compare("PAS"))
    {
      de = new PASDescriptor;
    }
#endif
else    assert(0);
    return de;
  }
  //default we dont use matcher, because we use flann index
DescriptorMatcher* TrainingSet::createDescriptorMatcher(const string& matcherType)
{
  DescriptorMatcher* dm = 0;
  if (!matcherType.compare("BF"))
    dm = new BruteForceMatcher<L1<float> > ;
#ifdef PAS_FEATURES
    else if (!matcherType.compare("PAS"))
    {
      dm = new BruteForceMatcher<PASDissimilarity<float> >;
    }
#endif
else    assert(0);
    return dm;
  }


void TrainingSet::init()
{
#ifdef PAS_FEATURES
  detector = TrainingSet::createDetector(string("PAS"));
  descriptorExtractor = TrainingSet::createDescriptorExtractor(string("PAS"));
  descriptorMatcher = TrainingSet::createDescriptorMatcher(string("PAS"));
#else
  //create detector, descriptor
  detector = createDetector(params.detectorName);
  descriptorExtractor = createDescriptorExtractor(params.descriptorName);
#endif
  isDrawInliers = true;
  isDrawCorrespondence = false;
  isPrintResults = true;
  isPrintTiming = true;
  isDrawProjection = true;
  isDrawClusters = false;
  isSaveImages = false;
  isNode = true;
}

//dir - directory with training base
TrainingSet::TrainingSet(string dir, string configPath)
{
  if (!configPath.empty())
  {
    params.readParameters(configPath);
  }

  init();

  if (dir.empty())
    return;

  //readCamModel(dir + "/" + "info.txt", camModel);
  //read images and point clouds for every object
  cout << "reading objects" << flush;
  readObjects("config.txt", dir);
  cout << "...end" << endl;
  cout << flush;

  //init objNum and maxImgNum
  objNum = objects.size();

  cout << "Read " << objNum << " objects" << endl;

  maxImgNum = -1;
  for (int i = 0; i < objNum; i++)
  {
    if (maxImgNum < objects[i]->imagesNum)
      maxImgNum = objects[i]->imagesNum;
  }

  //merge descriptors into one matrix
  concatenateMatrices();

  //create flann index
  flann_index = new cv::flann::Index(descriptors, cv::flann::KDTreeIndexParams(4));
  // using 4 randomized kdtrees

  colors.resize(objNum);
  for (int i = 0; i < objNum; i++)
  {
    colors[i] = Scalar(rand() % 255, rand() % 255, rand() % 255);
    float max_color = max(colors[i].val[0], max(colors[i].val[1], colors[i].val[2]));
    float factor = 255.0/max_color;
    colors[i] *= factor;
  }

#ifdef PAS_FEATURES
  vector<ContourSegmentNetwork*> csns;
  for( size_t objIdx = 0; objIdx < objects.size(); objIdx++ )
  {
    Object* obj = objects[objIdx];
    for (size_t imgIdx = 0; imgIdx < obj->images.size(); imgIdx++)
    {
      csns.push_back( &obj->csns[imgIdx] );
    }
  }

  //#define CONSTRUCT_CODEBOOK
#ifdef CONSTRUCT_CODEBOOK
  const double dissimilarityThreshold = GET_DISSIMILARITY_THRESHOLD();
  constructCodebook( csns, codebook, dissimilarityThreshold );
  //FileStorage fsRead( "codebook.xml", FileStorage::READ );
  //fsRead[ "codebook" ] >> codebook;
  //fsRead.release();


  //calcIDF( csns, codebook, idf, dissimilarityThreshold );
  idf.create( 1, codebook.rows, CV_32FC1 );
  for( int bin=0;bin<=codebook.rows;bin++ )
  {
    PAS::PASDescriptor codeword = codebook.ptr<float>( bin );

    int ni = 0;
    int objNi = 0;
    int n = 0;
    for( size_t objIdx = 0; objIdx < objects.size(); objIdx++ )
    {
      bool isObjNi = false;
      Object* obj = objects[objIdx];
      n += obj->images.size();
      for (size_t imgIdx = 0; imgIdx < obj->images.size(); imgIdx++)
      {
        for( size_t j=0;j<obj->csns[imgIdx].PASes.size();j++ )
        {
          double distance = PAS::PASDescriptor::dissimilarity( codeword, obj->csns[imgIdx].PASes[j].descriptor );
          if( distance < dissimilarityThreshold )
          {
            ni++;
            if( !isObjNi )
            {
              objNi++;
              isObjNi = true;
            }
            break;
          }
        }
      }
    }
    //idf.at<float>( 0, bin ) = log( n / ( (float) ni ) ) + log( objects.size() / ( (float) objNi ) );
    //idf.at<float>( 0, bin ) = ( objects.size() / ( (float) objNi ) );

    //TODO: very controversial
    //idf.at<float>( 0, bin ) = log( n / ( (float) ni ) ) + log( objects.size() / ( (float) objNi ) );
    //idf.at<float>( 0, bin ) = log( ni / ( (float) n ) ) + log( objects.size() / ( pow( (float) objNi, 1.5 ) ) );
    //idf.at<float>( 0, bin ) = log( ni / ( pow( (float) objNi, 1.5 ) ) ) + 0.5 * log( objects.size() );

    //idf.at<float>( 0, bin ) = ( ni / ( pow( (float) objNi, 1.5 ) ) ) * ( ni / ( pow( (float) objNi, 1.5 ) ) );


    idf.at<float>( 0, bin ) = 1;
    //idf.at<float>( 0, bin ) = codeword.length1 + codeword.length2;
  }

  sortCodebook( codebook, idf );

  FileStorage fs( "codebook.xml", FileStorage::WRITE );
  fs << "codebook" << codebook;
  fs << "idf" << idf;
#else
  FileStorage fs( "codebook.xml", FileStorage::READ );
  fs[ "codebook" ] >> codebook;
  fs[ "idf" ] >> idf;
#endif

  //visualizeCodebook( codebook, idf );

#endif
}

//filename - config file name
//dir - directory with training base
void TrainingSet::readObjects(string filename, string dir)
{
  ifstream file;
  string filepath = dir + "/" + filename;
  cout << filepath << endl;
  file.open(filepath.c_str());
  if (file.is_open() == false)
  {
    cout << "File " << filepath << " not found, exiting!" << endl;
    exit(-1);
  }

  //reading "config.txt" file
  //file format:
  //object_name images_count
  //...
  sensor_msgs::CameraInfo ci;
  while (!file.eof())
  {
    string name;
    int imageCount;
    file >> name >> imageCount;
    if (name.length() == 0)
      continue;

    cout << "Reading object " << name << "...";
    cout << flush;
    image_geometry::PinholeCameraModel* cm;
    readCamModel(dir + "/" + name + "/info.txt", ci, cm);
    //in Object constructor we find keypoints for t and compute descriptors
    //also we filter keypoints using point cloud (we use only keypoints, we know 3d points)
    objects.push_back(new Object(name, dir + "/" + name + "/", imageCount, cm, (const FeatureDetector*)detector,
                                 (const DescriptorExtractor*)descriptorExtractor, params.isRecalcDescriptors));
    delete cm;
    cout << "done" << endl;
    cout << flush;

  }
  file.close();
  ROS_INFO("Training base reading is complete.");
}

//read CameraInfo message from text file
void TrainingSet::readCamModel(string filename, sensor_msgs::CameraInfo& ci,
                               image_geometry::PinholeCameraModel*& cam_model)
{
  ifstream file;
  file.open(filename.c_str());

  file >> ci.height >> ci.width;
  for (int i = 0; i < 5; i++)
  {
    double d;
    file >> d;
    ci.D.push_back(d);
  }
  for (int i = 0; i < 9; i++)
    file >> ci.K[i];
  for (int i = 0; i < 9; i++)
    file >> ci.R[i];
  for (int i = 0; i < 12; i++)
    file >> ci.P[i];
  file.close();

  cam_model = new image_geometry::PinholeCameraModel();
  cam_model->fromCameraInfo(ci);
}

void TrainingSet::initTestCamModel(std::string filename)
{
  image_geometry::PinholeCameraModel* cam_model;
  sensor_msgs::CameraInfo ci;
  readCamModel(filename, ci, cam_model);
  cameraMatrix.create(3, 3, CV_32FC1);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cameraMatrix.at<float> (i, j) = ci.K[j + 3 * i];

  distCoeffs.create(1, 5, CV_32FC1);
  for (int j = 0; j < 5; j++)
    distCoeffs.at<float> (0, j) = ci.D[j];
  delete cam_model;
}

void TrainingSet::setTestCameraInfo(sensor_msgs::CameraInfoConstPtr cameraInfo)
{
  cameraMatrix.create(3, 3, CV_32FC1);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cameraMatrix.at<float> (i, j) = cameraInfo->K[j + 3 * i];

  distCoeffs.create(1, 5, CV_32FC1);
  for (int j = 0; j < 5; j++)
    distCoeffs.at<float> (0, j) = cameraInfo->D[j];

  //cout << cameraMatrix << distCoeffs << endl;
}

//merge descriptors for all train images into one matrix
//init obj_indices, img_indices, keypoint_indices
void TrainingSet::concatenateMatrices()
{
  cout << "Desc" << objects[0]->descriptors[0].rowRange(1, 2) << endl;
  obj_indices.clear();
  img_indices.clear();
  keypoint_indices.clear();
  int rowsNumber = 0;
  int cols = objects[0]->descriptors[0].cols;
  for (int i = 0; i < objNum; i++)
    for (int j = 0; j < objects[i]->imagesNum; j++)
      rowsNumber += objects[i]->descriptors[j].rows;
  descriptors.create(rowsNumber, cols, CV_32FC1);
  obj_indices.reserve(rowsNumber);
  img_indices.reserve(rowsNumber);
  keypoint_indices.reserve(rowsNumber);
  int rowIndex = 0;
  for (int i = 0; i < objNum; i++)
    for (int j = 0; j < objects[i]->imagesNum; j++)
    {
      for (int k = 0; k < objects[i]->descriptors[j].rows; k++)
      {
        Mat row(1, descriptors.cols, descriptors.type(), descriptors.ptr(rowIndex++));
        objects[i]->descriptors[j].row(k).copyTo(row);
        obj_indices.push_back(i);
        img_indices.push_back(j);
        keypoint_indices.push_back(k);
      }
      objects[i]->descriptors[j] = Mat();
    }
  FileStorage fs;
    fs.open("indices.yaml", FileStorage::WRITE);
    fs << "obj" << Mat(obj_indices);
    fs << "img" << Mat(img_indices);
    fs << "kp" << Mat(keypoint_indices);
    fs.release();
}

//hierarchical clustering of points from test image than matched to different images of one object
//imagePoints - coord of points
//imageIndexes - image indexes in train base (we prohibit points matched to different images in one cluster, so distance between them is infinity)
//maxDist - max distance for clustering
//clusterIndixes - result indexes for every cluster
void TrainingSet::clusterPoints(const vector<Point2f>& imagePoints, const vector<int>& imageIndexes, float maxDist,
                                vector<vector<int> >& clusterIndixes)
{
  list<Cluster> clusters;
  //init clusters
  for (size_t i = 0; i < imagePoints.size(); i++)
  {
    clusters.push_back(Cluster(imagePoints[i], i, imageIndexes[i]));
  }

  //merge
  bool isMergeAnyClusters;
  do
  {
    isMergeAnyClusters = false;
    for (list<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
    {
      double min_dist = std::numeric_limits<double>::max(), dist;
      list<Cluster>::iterator cur_min = it;

      list<Cluster>::iterator next = it;
      next++;
      for (; next != clusters.end(); next++)
      {
        //#ifndef CHESSBOARD
        if ((*it).imgInd != (*next).imgInd)
          dist = std::numeric_limits<double>::max();
        else
          //#endif
          dist = norm((*it).center - (*next).center);
        if (dist < min_dist)
        {
          min_dist = dist;
          cur_min = next;
        }
      }

      if (cur_min != it && min_dist < maxDist)
      {
        //merge it and next clusters
        isMergeAnyClusters = true;
        for (size_t i = 0; i < ((*cur_min).pIndices.size()); i++)
        {
          (*it).pIndices.push_back((*cur_min).pIndices[i]);
        }
        (*it).sumx += (*cur_min).sumx;
        (*it).sumy += (*cur_min).sumy;
        (*it).center = Point2f((*it).sumx / (*it).pIndices.size(), (*it).sumy / (*it).pIndices.size());
        clusters.erase(cur_min);
      }
    }
  } while (isMergeAnyClusters);

  for (list<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    vector<int> indices;
    for (size_t j = 0; j < (*it).pIndices.size(); j++)
    {
      indices.push_back((*it).pIndices[j]);
    }
    clusterIndixes.push_back(indices);
  }
}

void TrainingSet::publishPoints(const vector<Point3f>& points, const ros::Publisher &points_pub,
                                const vector<int>& indices)
{
  vector<Scalar> colors;
  assert(points.size() == indices.size());
  if (points.size() == 0)
  {
    return;
  }

  int objNum = *std::max_element(indices.begin(), indices.end()) + 1;
  colors.resize(objNum);
  for (int i = 0; i < objNum; i++)
  {
    colors[i] = Scalar(rand() % 255, rand() % 255, rand() % 255);
    printf("color[%d] = %f %f %f\n", i, colors[i].val[0], colors[i].val[1], colors[i].val[2]);
  }

  visualization_msgs::Marker cammark;
  cammark.header.frame_id = "/high_def_frame";
  cammark.header.stamp = ros::Time();
  cammark.ns = "high_def_frame";
  cammark.id = 0;
  cammark.action = visualization_msgs::Marker::ADD;
  cammark.pose.position.x = 0;
  cammark.pose.position.y = 0;
  cammark.pose.position.z = 0;
  cammark.pose.orientation.x = 0.0;
  cammark.pose.orientation.y = 0.0;
  cammark.pose.orientation.z = 0.0;
  cammark.pose.orientation.w = 1.0;
  cammark.scale.x = 0.01;
  cammark.scale.y = 0.01;
  cammark.scale.z = 0.01;
  cammark.color.a = 1.0f;
  cammark.lifetime = ros::Duration();
  cammark.type = visualization_msgs::Marker::POINTS;
  cammark.points.resize(points.size());
  cammark.colors.resize(points.size());
  const float scale = 20.0f;
  for (size_t m = 0; m < points.size(); m++)
  {
    cammark.points[m].x = points[m].x * scale;
    cammark.points[m].y = points[m].z * scale;
    cammark.points[m].z = -points[m].y * scale;

    cammark.colors[m].r = float(colors[indices[m]].val[2]) / (float)255;
    cammark.colors[m].g = colors[indices[m]].val[1] / (float)255;
    cammark.colors[m].b = colors[indices[m]].val[0] / (float)255;
    cammark.colors[m].a = 1.0;
  }
  points_pub.publish(cammark);
}

//publish pose for found object
void TrainingSet::publishPoints(const vector<Point3f>& points, const ros::Publisher &points_pub, const int index)
{
  visualization_msgs::Marker cammark;
  cammark.header.frame_id = "/high_def_frame";
  cammark.header.stamp = ros::Time();
  cammark.ns = "high_def_frame";
  cammark.id = index;
  cammark.action = visualization_msgs::Marker::ADD;
  cammark.pose.position.x = 0;
  cammark.pose.position.y = 0;
  cammark.pose.position.z = 0;
  cammark.pose.orientation.x = 0.0;
  cammark.pose.orientation.y = 0.0;
  cammark.pose.orientation.z = 0.0;
  cammark.pose.orientation.w = 1.0;
  cammark.scale.x = 0.2;
  cammark.scale.y = 0.2;
  cammark.scale.z = 0.2;
  cammark.color.r = colors[index].val[2] / (float)255;
  cammark.color.g = colors[index].val[1] / (float)255;
  cammark.color.b = colors[index].val[0] / (float)255;
  cammark.color.a = 1.0f;
  cammark.lifetime = ros::Duration();
  cammark.type = visualization_msgs::Marker::POINTS;
  cammark.points.resize(points.size());
  const float scale = 20.0f;
  for (size_t m = 0; m < points.size(); m++)
  {
    cammark.points[m].x = points[m].x * scale;
    cammark.points[m].y = points[m].y * scale;
    cammark.points[m].z = points[m].z * scale;
  }
  points_pub.publish(cammark);
}

void TrainingSet::detectAndMatchPoints(const cv::Mat& img, std::vector<cv::KeyPoint>& testKeypoints,
                                       std::vector<DMatch>& matches)
{
  //detect keypoints for test image
  int64 _t1 = cvGetTickCount();
  detector->detect(img, testKeypoints);
  int64 _t2 = cvGetTickCount();

  Mat testDescriptors;
  //compute descriptors for test image
  descriptorExtractor->compute(img, testKeypoints, testDescriptors);
  int64 _t3 = cvGetTickCount();
  cout << "Extracted " << testKeypoints.size() << " points" << endl;

  //match test image descriptors to train base descriptors
  cvflann::set_distance_type(cvflann::FLANN_DIST_MANHATTAN, 0);
  Mat m_indices(testDescriptors.rows, params.knnNum, CV_32S);
  Mat m_dists(testDescriptors.rows, params.knnNum, CV_32F);
  flann_index->knnSearch(testDescriptors, m_indices, m_dists, params.knnNum, cv::flann::SearchParams(64)); // maximum number of leafs checked
  int* indices_ptr = m_indices.ptr<int> (0);
  float* dists_ptr = m_dists.ptr<float> (0);
  int first_index, second_index, j;

  for (int i = 0; i < m_indices.rows; ++i)
  {
    for (int objId = 0; objId < objNum; objId++)
    {
      first_index = second_index = -1;
      for (j = 0; j < params.knnNum; j++)
      {
        if (obj_indices[indices_ptr[i * params.knnNum + j]] == objId)
        {
          first_index = j++;
          break;
        }
      }

      if (first_index < 0)
        continue;

      for (; j < params.knnNum; j++)
      {
        if (obj_indices[indices_ptr[i * params.knnNum + j]] == objId)
        {
          second_index = j;
          break;
        }
      }

      if (second_index == -1)
      {
        if (first_index < params.knnNum - 1)
          second_index = first_index + 1;
        else
          second_index = first_index;
      }

      if (dists_ptr[params.knnNum * i + first_index] < params.ratioTestThreshold * dists_ptr[params.knnNum * i
          + second_index])
      {
        DMatch match;
        match.queryIdx = i;
        match.trainIdx = indices_ptr[i * params.knnNum];
        if (match.queryIdx == 10)
                {
                  cout << "QUERY" << match.trainIdx << " " << img_indices[indices_ptr[i * params.knnNum]] << " " << objId << endl;
                }
        matches.push_back(match);
      }
    }
  }
  int64 _t4 = cvGetTickCount();
  printf("Timing:\n  keypoints %f\n  descriptors %f\n  matching %f\n", float(_t2 - _t1) / cvGetTickFrequency() * 1e-6,
         float(_t3 - _t2) / cvGetTickFrequency() * 1e-6, float(_t4 - _t3) / cvGetTickFrequency() * 1e-6);

  if (isPrintResults)
    cout << "Matched " << matches.size() << " points" << endl;
}

void TrainingSet::printMatchingResults(const vector<DMatch>& matches)
{
  // set up <objInd>, which holds the feature count for each model view
  // odd indexing: each object is a vector of <maxImgNum> size, where <maxImgNum> 
  //   is the max number of views for an object in the training set
  vector<int> objInd(objNum * maxImgNum, 0);
  int maxMatch = 0, maxMatchObj = 0;
  for (size_t ind = 0; ind < matches.size(); ind++)
  {
    objInd[obj_indices[matches[ind].trainIdx] * maxImgNum + img_indices[matches[ind].trainIdx]]++;
  }

  // find max match
  for (size_t i = 0; i < objInd.size(); i++)
    if (objInd[i] > maxMatch)
    {
      maxMatch = objInd[i];
      maxMatchObj = i / maxImgNum;
    }

  cout << "Total matches: " << matches.size() << endl;
  cout << "Max match is " << maxMatch << " for object " << maxMatchObj << endl;
  cout << "Matches for each object (row) per view (column)" << endl;
  for (int i = 0; i < objNum; i++)
  {
    cout << "Object " << i << ":" << "\t";
    for (int j = 0; j < objects[i]->imagesNum; j++)
      cout << objInd[i * maxImgNum + j] << "\t";
    cout << endl;
  }
}

void TrainingSet::drawProjectedPoints(const vector<Point3f>& _rotated_object_points, int objIndex)
{
  vector<Point3f> rotated_object_points = _rotated_object_points;
  filterOutlierPoints(rotated_object_points);

  vector<Point2f> projected_points;
  projected_points.resize(rotated_object_points.size());
  Mat rvec1, tvec1;
  rvec1.create(3, 1, CV_64FC1);
  tvec1.create(3, 1, CV_64FC1);
  rvec1.at<double> (0, 0) = rvec1.at<double> (1, 0) = rvec1.at<double> (2, 0) = 0.0;
  tvec1.at<double> (0, 0) = tvec1.at<double> (1, 0) = tvec1.at<double> (2, 0) = 0.0;
  projectPoints(Mat(rotated_object_points), rvec1, tvec1, cameraMatrix, distCoeffs, projected_points);
  if (isDrawProjection)
  {
    for (size_t mm = 0; mm < projected_points.size(); mm++)
    {
      circle(projImg, Point((int)projected_points[mm].x, (int)projected_points[mm].y), 1, colors[objIndex]);
    }

    vector<Point2f> hull;
    convexHull(Mat(projected_points), hull);

    vector<Point> ihull;
    for(size_t i = 0; i < hull.size(); i++)
    {
      ihull.push_back(Point(hull[i].x, hull[i].y));
    }

    vector<vector<Point> > _hull;
    _hull.push_back(ihull);

    drawContours(projImg, _hull, -1, colors[objIndex], 4);
    putText(projImg, objects[objIndex]->name.c_str(), ihull[0],
            FONT_HERSHEY_SIMPLEX, 2.0, colors[objIndex], 4);
  }

  Rect r = boundingRect(Mat(projected_points));
  rectangle(bbProjImg, Point(r.x, r.y), Point(r.x + r.width, r.y + r.height), colors[objIndex], 4);
  putText(bbProjImg, objects[objIndex]->name.c_str(), Point(r.x, r.y),
          FONT_HERSHEY_SIMPLEX, 2.0, colors[objIndex], 4);
}

void cross(Mat& img, Point p, Scalar color, int size)
{
  if (p.x < size || p.y < size || p.x + size >= img.cols || p.y + size >= img.rows)
    return;
  line(img, Point(p.x - size, p.y), Point(p.x + size, p.y), color);
  line(img, Point(p.x, p.y - size), Point(p.x, p.y + size), color);
}

void TrainingSet::findPoseGuesses(Object* object, const vector<KeyPoint>& keypoints, const vector<DMatch>& obj_matches,
                                  const vector<vector<int> >& indices, const Mat& img, vector<PoseGuess>& poseGuesses)
{
  // loop over clusters, doing RANSAC on each
  for (size_t k = 0; k < indices.size(); k++)
  {
    vector<Point2f> imagePoints;
    vector<Point2f> objPoints;
    vector<Point3f> objectPoints;
    vector<int> imageIndexes;
    vector<int> globalIndices;

#ifndef CHESSBOARD
    int img_ind = img_indices[obj_matches[indices[k][0]].trainIdx];

    //we filter small cluster
    if ((int)indices[k].size() < params.minClusterSize)
    continue;

    int l;
    for (size_t m = 0; m < indices[k].size(); m++)
    {
      l = indices[k][m];
      int ind = object->pointsIndices[img_ind][keypoint_indices[obj_matches[l].trainIdx]];
      imagePoints.push_back(Point2f(keypoints[obj_matches[l].queryIdx].pt.x,
              keypoints[obj_matches[l].queryIdx].pt.y));
      objPoints.push_back(Point2f(object->keypoints[img_ind][keypoint_indices[obj_matches[l].trainIdx]].pt.x,
              object->keypoints[img_ind][keypoint_indices[obj_matches[l].trainIdx]].pt.y));
      objectPoints.push_back(Point3f(object->clouds[img_ind].points[ind].x, object->clouds[img_ind].points[ind].y,
              object->clouds[img_ind].points[ind].z));
      globalIndices.push_back(obj_matches[l].queryIdx);
    }
#else
    //we filter small cluster
    if ((int)indices[k].size() < params.minClusterSize)
      continue;
    cout << endl << "cluster " << indices[k].size() << endl;
    vector<int> activePoints;
    activePoints.assign(indices[k].size(), 1);

    Mat drawProjImg;
    if (!object->images[0].empty())
      object->images[0].copyTo(drawProjImg);
    while (*std::max_element(activePoints.begin(), activePoints.end()) > 0)
    {
      imagePoints.clear();
      objPoints.clear();
      objectPoints.clear();
      imageIndexes.clear();
      globalIndices.clear();

      int l;

      vector<int> acivePointsIndices;

      for (size_t m = 0; m < indices[k].size(); m++)
      {
        if (activePoints[m] == 0)
          continue;

        acivePointsIndices.push_back(m);
        int img_ind = img_indices[obj_matches[indices[k][m]].trainIdx];
        l = indices[k][m];
        int ind = object->pointsIndices[img_ind][keypoint_indices[obj_matches[l].trainIdx]];

        imagePoints.push_back(Point2f(keypoints[obj_matches[l].queryIdx].pt.x, keypoints[obj_matches[l].queryIdx].pt.y));
        globalIndices.push_back(obj_matches[l].queryIdx);
        objPoints.push_back(Point2f(object->keypoints[img_ind][keypoint_indices[obj_matches[l].trainIdx]].pt.x,
                                    object->keypoints[img_ind][keypoint_indices[obj_matches[l].trainIdx]].pt.y));
        imageIndexes.push_back(img_ind);
        vector<Point3f> point;
        point.push_back(Point3f(object->clouds[img_ind].points[ind].x, object->clouds[img_ind].points[ind].y,
                                object->clouds[img_ind].points[ind].z));
        vector<Point3f> rotated_point;
        project3dPoints(point, object->rvecs[img_ind], object->tvecs[img_ind], rotated_point);
        objectPoints.push_back(rotated_point[0]);

        if (img_ind == 0)
        {
          vector<Point2f> projectedPoint;
          projectPoints(Mat(point), object->rvecs[img_ind], object->tvecs[img_ind], cameraMatrix, distCoeffs,
                        projectedPoint);
          if (!drawProjImg.empty())
            circle(drawProjImg, Point(projectedPoint[0].x, projectedPoint[0].y), 5, cvScalar(0));
        }
      }

      //      printf("Running cluster %d with %d points\n", (int)k, (int)objectPoints.size());

      if (isDrawInliers && !isNode)
      {
        if (!isSaveImages && !drawProjImg.empty())
        {
          namedWindow("projection img1", 1);
          imshow("projection img1", drawProjImg);
        }
        else
        {
          char imageName[1024];
          sprintf(imageName, "%s_inlier_cloudproj_%04d.jpg", testName.c_str(), (int)poseGuesses.size());
          //          imwrite(imageName, drawProjImg);
        }
      }
      //      waitKey(0);
#endif

      vector<int> inliers;
      Mat rvec, tvec;
      //run solvePnPRansac for finding inliers (using point clouds from train base)
      solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, RANSAC_ITERATIONS,
                     params.reprojectErrorThreshold, 100, &inliers);

      if (isPrintResults)
      {
        //      cout << "Inliers count = " << inliers.size() << endl;
      }

#ifdef CHAMFER_PLUS_TEXTURE
      vector<Point3f> points;
      for( size_t edgelIdx=0;edgelIdx<object->edgels[img_ind].size();edgelIdx++ )
      {
        int pointIdx = object->edgelsIndices[img_ind][edgelIdx];
        const pcl::PointXYZ &pointXYZ = object->clouds[img_ind].points.at( pointIdx );
        points.push_back( Point3f( pointXYZ.x, pointXYZ.y, pointXYZ.z ) );
      }

      GMap3D gmap(rvec, tvec, cameraMatrix, distCoeffs);
      GMatchCM gMatchEdges;
      gMatchEdges.setTemplateEdgels(points);
      gMatchEdges.setTestImage(img);
      float cost = gMatchEdges.calcMatchCost(gmap);

      if( isPrintResults )
      cout << "Chamfer matching cost = " << cost << endl;

      if( isDrawProjection )
      {
        gMatchEdges.showMatch(gmap);
        waitKey(0);
      }
#endif

      //we found max inliers count for clusters of one object (we assume that there is only one copy of any objects in test image)
      const int roughMinInliersCount = params.minInliersCount / 2;
      if ((int)inliers.size() > roughMinInliersCount)
      {
        PoseGuess guess;
#ifndef CHESSBOARD
        guess.img_idx = img_ind;
#else
        guess.imageIndexes = imageIndexes;
        guess.img_idx = 0;
#endif
        /*
         bestInliers.push_back(inliers);
         bestObjPoints.push_back(objPoints);
         bestImagePoints.push_back(imagePoints);
         bestObjectPoints.push_back(objectPoints);
         Mat _rvec, _tvec;
         rvec.copyTo(_rvec);
         tvec.copyTo(_tvec);
         rvecl.push_back(_rvec);
         tvecl.push_back(_tvec);
         */
        rvec.copyTo(guess.rvec);
        tvec.copyTo(guess.tvec);
        guess.inliers = inliers;
        guess.objPoints = objPoints;
        guess.imagePoints = imagePoints;

        poseGuesses.push_back(guess);

        for (size_t m = 0; m < inliers.size(); m++)
        {
          //          cout << "remove point " << inliers[m] << endl;
          activePoints[acivePointsIndices[inliers[m]]] = 0; //removing the points for which the pose has been found
        }

#if 1
        Mat drawImg;
        img.copyTo(drawImg);

        vector<Point2f> projectedPoints;
        projectPoints(Mat(objectPoints), rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
        float sum = 0;
        for (size_t i = 0; i < inliers.size(); i++)
        {
          float dist = norm(imagePoints[inliers[i]] - projectedPoints[inliers[i]]);
          int img_ind = imageIndexes[inliers[i]];
          //          printf("image index %d, dist = %f\n", img_ind, dist);
          if (img_ind == 0)
          {
            //            printf("point %d %f %f %f projects into %f %f\n", inliers[i], objectPoints[inliers[i]].x, objectPoints[inliers[i]].y, objectPoints[inliers[i]].z,
            //                 projectedPoints[inliers[i]].x, projectedPoints[inliers[i]].y);
          }
          sum += dist * dist;

          //        if (img_ind == 0)
          {
            circle(drawImg, Point(projectedPoints[inliers[i]].x, projectedPoints[inliers[i]].y), 5, Scalar(255));
            cross(drawImg, Point(imagePoints[inliers[i]].x, imagePoints[inliers[i]].y), Scalar(0), 5);
          }
        }
        printf("reprojection error: %f\n", sqrt(sum / inliers.size()));

        Mat drawImgS;
        resize(drawImg, drawImgS, Size(drawImg.cols * params.scale, drawImg.rows * params.scale));
        if (isDrawInliers && !isNode)
        {
          if (!isSaveImages)
          {
            namedWindow("inliers 0", 1);
            imshow("inliers 0", drawImgS);
          }
          else
          {
            char imageName[1024];
            sprintf(imageName, "%s_inlier_proj_%04d.jpg", testName.c_str(), (int)poseGuesses.size());
            imwrite(imageName, drawImg);
          }
        }

        vector<Point3f> points, points0;
        if (object->clouds.size() > 1)
        {
          for (size_t i = 0; i < object->clouds[1].points.size(); i++)
          {
            points.push_back(Point3f(object->clouds[1].points[i].x, object->clouds[1].points[i].y,
                                     object->clouds[1].points[i].z));
          }

          for (size_t i = 0; i < object->clouds[0].points.size(); i++)
          {
            points0.push_back(Point3f(object->clouds[0].points[i].x, object->clouds[0].points[i].y,
                                      object->clouds[0].points[i].z));
          }

          vector<Point3f> _points, _points0;
          vector<Point2f> projectedPoints0;
          project3dPoints(points, object->rvecs[1], object->tvecs[1], _points);
          project3dPoints(points0, object->rvecs[0], object->tvecs[0], _points0);

          projectPoints(Mat(_points), rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
          projectPoints(Mat(_points0), rvec, tvec, cameraMatrix, distCoeffs, projectedPoints0);

          img.copyTo(drawImg);
          for (size_t i = 0; i < projectedPoints.size(); i++)
          {
            circle(drawImg, Point(projectedPoints[i].x, projectedPoints[i].y), 5, cvScalar(0));
          }
          resize(drawImg, drawImgS, Size(drawImg.cols * params.scale, drawImg.rows * params.scale));

          if (isDrawInliers && !isNode)
          {
            namedWindow("pointcloud 1", 1);
            imshow("pointcloud 1", drawImgS);
          }

          img.copyTo(drawImg);
          for (size_t i = 0; i < projectedPoints0.size(); i++)
          {
            circle(drawImg, Point(projectedPoints0[i].x, projectedPoints0[i].y), 5, cvScalar(0));
          }
          resize(drawImg, drawImgS, Size(drawImg.cols * params.scale, drawImg.rows * params.scale));

          if (isDrawInliers && !isNode)
          {
            namedWindow("pointcloud 0", 1);
            imshow("pointcloud 0", drawImgS);
          }
        }
#endif

      }
      else
      {
        break;
      }
    }
#if defined(CHESSBOARD)
  }
#endif

}

void TrainingSet::refinePoses(const Object* object, const vector<KeyPoint>& keypoints, const vector<DMatch>& matches,
                              const Mat& img, vector<PoseGuess>& guesses)
{
  vector<Point2f> imagePoints;
  vector<Point3f> objectPoints;
  vector<Point2f> objPoints;
  imagePoints.resize(matches.size());
  objectPoints.resize(matches.size());
  objPoints.resize(matches.size());
  for (size_t i = 0; i < matches.size(); i++)
  {
    int img_ind = img_indices[matches[i].trainIdx];
    int point_ind = object->pointsIndices[img_ind][keypoint_indices[matches[i].trainIdx]];

    imagePoints[i] = Point2f(keypoints[matches[i].queryIdx].pt.x, keypoints[matches[i].queryIdx].pt.y);
    objPoints[i] = Point2f(object->keypoints[img_ind][keypoint_indices[matches[i].trainIdx]].pt.x,
                           object->keypoints[img_ind][keypoint_indices[matches[i].trainIdx]].pt.y);
    vector<Point3f> point;
    point.push_back(Point3f(object->clouds[img_ind].points[point_ind].x, object->clouds[img_ind].points[point_ind].y,
                            object->clouds[img_ind].points[point_ind].z));
    vector<Point3f> rotated_point;
    project3dPoints(point, object->rvecs[img_ind], object->tvecs[img_ind], rotated_point);
    objectPoints[i] = rotated_point[0];
  }

  for (size_t i = 0; i < guesses.size(); i++)
  {
    // find inliers in global match array
    vector<Point2f> projectedPoints;
    projectPoints(Mat(objectPoints), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);

    vector<Point2f> inlierImagePoints;
    vector<Point3f> inlierObjectPoints;
    vector<int> inlierIndices;
    for (size_t j = 0; j < imagePoints.size(); j++)
    {
      float dist = norm(imagePoints[j] - projectedPoints[j]);
      if (dist < params.reprojectErrorThreshold * 5)
      {
        inlierImagePoints.push_back(imagePoints[j]);
        inlierObjectPoints.push_back(objectPoints[j]);
        inlierIndices.push_back(j);
      }
    }

#if 0
    vector<Point2f> hull;
    convexHull(Mat(inlierImagePoints), hull);
    float stddev = contourArea(Mat(hull));
#else
    float stddev = computeStddev(inlierObjectPoints);
#endif

    vector<int> inliers;
    solvePnPRansac(inlierObjectPoints, inlierImagePoints, cameraMatrix, distCoeffs, guesses[i].rvec, guesses[i].tvec,
                   true, RANSAC_ITERATIONS * 10, params.reprojectErrorThreshold, params.minInliersCount, &inliers);
    guesses[i].fineInliersCount = inliers.size();
    // copy inlier indices into the global point array
    guesses[i].inliers.resize(inliers.size());
    for (size_t j = 0; j < inliers.size(); j++)
    {
      guesses[i].inliers[j] = inlierIndices[inliers[j]];
    }
    guesses[i].imagePoints = imagePoints;
    guesses[i].objPoints = objPoints;
    guesses[i].img_idx = 0;
    guesses[i].stddev = stddev;

    cout << "Refine, OBJECT NAME = " << object->name << endl;

    if (isPrintResults)
    {
      cout << "Guess " << i << ":" << guesses[i].inliers.size() << " rough inliers, " << inliers.size()
          << " refined inliers" << ", cluster stddev ratio " << stddev / object->stddev << endl;
    }

    if (isDrawInliers)
    {
      Mat drawImg(img.rows, img.cols, CV_8UC3);
      cvtColor(img, drawImg, CV_GRAY2RGB);
      for (size_t i = 0; i < inliers.size(); i++)
      {
        circle(drawImg, Point(inlierImagePoints[inliers[i]].x, inlierImagePoints[inliers[i]].y), 5,
               cvScalar(0, 0, 255), -1);
      }

      Mat drawImgS;
      resize(drawImg, drawImgS, Size(drawImg.cols * params.scale, drawImg.rows * params.scale));

      if (!isSaveImages)
      {
        namedWindow("fine inliers", 1);
        imshow("fine inliers", drawImgS);
        waitKey(0);
      }
      else
      {
        char imageName[1024];
        sprintf(imageName, "%s_fine_inliers_%04d.jpg", testName.c_str(), (int)guesses.size());
        imwrite(imageName, drawImg);
      }
    }
  }
}

void TrainingSet::filterOverlappingPoses(const vector<Point2f>& imagePoints, vector<PoseGuess>& guesses)
{
  vector<HCluster> clusters;
  for (size_t i = 0; i < guesses.size(); i++)
  {
//    HCluster cluster(guesses[i], &imagePoints, guesses[i].inliers);
    HCluster cluster(guesses[i], guesses[i].projectedPoints, guesses[i].inliers.size());
    clusters.push_back(cluster);
  }

  const float minCover = 0.5;
  filterClusters(clusters, minCover, projImg);

  guesses.clear();
  for (vector<HCluster>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    guesses.push_back(it->guess);
  }
}

void TrainingSet::nullCloudFilter(vector<PoseGuess>& guesses, const cv::Mat& img, const Object* object, const vector<Point2f> imgPoints)
{
   static Mat projectionMask(img.rows, img.cols, CV_8U, Scalar(0));

   //filter guesses with null points cloud
   vector<Point3f> nullCloudPoints;
   for (size_t m = 0; m < object->clouds[0].points.size(); m++)
   {
     nullCloudPoints.push_back(Point3f(object->clouds[0].points[m].x, object->clouds[0].points[m].y,
                                       object->clouds[0].points[m].z));
   }

  vector<PoseGuess> filteredWithNullCloud;
  for (size_t i = 0; i < guesses.size(); i++)
  {
    vector<Point2f> projectedPoints;
    projectPoints(Mat(nullCloudPoints), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);

    vector<Point2f> filterProjectedPoints;

    projectionMask.setTo(Scalar::all(0));
    for (size_t m = 0; m < projectedPoints.size(); m++)
    {
      if (projectedPoints[m].x < img.cols && projectedPoints[m].y < img.rows)
      {
        filterProjectedPoints.push_back(projectedPoints[m]);
        circle(projectionMask, projectedPoints[m], 5, Scalar(255), 3);
      }
    }

    transformMaskUsingBiggestContour(projectionMask, true);

    int count = 0;
    for (size_t m = 0; m < guesses[i].inliers.size(); m++)
    {
        if (projectionMask.at<uint8_t> (imgPoints[guesses[i].inliers[m]].y, imgPoints[guesses[i].inliers[m]].x) == 0)
          count++;
    }
    if (count < guesses[i].inliers.size() * 0.5)
    {
      filteredWithNullCloud.push_back(guesses[i]);
    }
    else if (isPrintResults)
    {
      cout << "Filtered with null points cloud!" << endl;
    }

    /*Mat drawImg;
    projectionMask.copyTo(drawImg);
    for (size_t m = 0; m < guesses[i].inliers.size(); m++)
    {
      circle(drawImg, imgPoints[guesses[i].inliers[m]], 5, Scalar(255));
    }
    Mat drawS;
    resize(drawImg, drawS, Size(), 0.3, 0.3);
    namedWindow("temp", 1);
    imshow("temp", drawS);*/
  }
  guesses = filteredWithNullCloud;
}

void TrainingSet::recognize(const cv::Mat img, std::vector<ObjectInfo>& res_objects, const ros::Publisher &points_pub)
{
  assert(maxImgNum != 0);
  //clear results vector
  res_objects.clear();

  cvtColor(img, bbProjImg, CV_GRAY2RGB);

  vector<KeyPoint> keypoints;
  vector<DMatch> matches;
  //detect keypoints, compute descriptors and match them
  int64 _t1 = cvGetTickCount();
  detectAndMatchPoints(img, keypoints, matches);
  int64 _t2 = cvGetTickCount();
  if (isPrintTiming)
    printf("detectAndMatchPoints: time elapsed %f\n", float(_t2 - _t1) / cvGetTickFrequency() * 1e-6);

  if (keypoints.empty())
    return;

  if (isPrintResults)
    printMatchingResults(matches);

  if (isDrawProjection)
  {
    cvtColor(img, projImg, CV_GRAY2RGB);
  }

  vector<Point3f> fullObjectPoints;
  vector<int> fullObjectIndices;

  // loop over all objects (??? why not just objects that have a good NN response)
  for (int objectIndex = 0; objectIndex < objNum; objectIndex++)
  {
    vector<Point2f> imagePoints;
    vector<Point2f> objPoints;
    vector<Point3f> objectPoints;
    vector<int> imageIndexes;
    vector<Mat> rvecl, tvecl;

    Object* object = objects[objectIndex];
    if (!object->rvecs.size())
      continue;

    printf("object %d, image 0: rvec = %f %f %f, tvec = %f %f %f\n", objectIndex, object->rvecs[0].at<double> (0, 0),
           object->rvecs[0].at<double> (1, 0), object->rvecs[0].at<double> (2, 0), object->tvecs[0].at<double> (0, 0),
           object->tvecs[0].at<double> (1, 0), object->tvecs[0].at<double> (2, 0));

    //we choose keypoints matched to one object
    vector<DMatch> obj_matches;
    vector<Point2f> imgPoints;
    vector<int> imgIndexes;
    for (vector<DMatch>::const_iterator it = matches.begin(); it < matches.end(); ++it)
    {
      if (obj_indices[it->trainIdx] == objectIndex)
      {
        imgPoints.push_back(Point2f(keypoints[it->queryIdx].pt.x, keypoints[it->queryIdx].pt.y));
        obj_matches.push_back(*it);
        imgIndexes.push_back(img_indices[it->trainIdx]);
      }
    }

    // get clusters, put into <indices> variable
    // each element of <indices> is one cluster
    vector<vector<int> > indices;

    int64 _t3 = cvGetTickCount();
    //cluster these points (distance between points matched to different images is infinity)
    clusterPoints(imgPoints, imgIndexes, params.clusterThreshold, indices);
    int64 _t4 = cvGetTickCount();
    if (isPrintTiming)
      printf("clusterPoints: time elapsed %f\n", float(_t4 - _t3) / cvGetTickFrequency() * 1e-6);

    if (isDrawClusters)
      drawClusters(img, imgPoints, indices);

    vector<PoseGuess> guesses;
    findPoseGuesses(object, keypoints, obj_matches, indices, img, guesses);
    int64 _t5 = cvGetTickCount();
    if (isPrintTiming)
      printf("findPoseGuesses: time elapsed %f\n", float(_t5 - _t4) / cvGetTickFrequency() * 1e-6);
    refinePoses(object, keypoints, obj_matches, img, guesses);
    int64 _t6 = cvGetTickCount();
    if (isPrintTiming)
      printf("refinePoses: time elapsed %f\n", float(_t6 - _t5) / cvGetTickFrequency() * 1e-6);

    // filter bad guesses
    cout << "Filtering guesses, object name = " << object->name << endl;
    vector<PoseGuess> filtered;
    for (size_t i = 0; i < guesses.size(); i++)
    {
      if (guesses[i].fineInliersCount < params.minInliersCount)
        continue;
      if (guesses[i].stddev < object->stddev * params.minStddevFactor || guesses[i].stddev > object->stddev)
      {
        cout << "Filtered inliers size, stddev = " << guesses[i].stddev << ", object stddev = " << object->stddev
            << ", div = " << guesses[i].stddev / object->stddev << endl;
        continue;
      }

      filtered.push_back(guesses[i]);
    }
    guesses = filtered;

    for (size_t i = 0; i < guesses.size(); i++)
    {
      vector<Point3f> points;
      for (size_t m = 0; m < object->clouds[guesses[i].img_idx].points.size(); m++)
      {
        if(rand() % object->clouds[guesses[i].img_idx].points.size() > 100) continue;

        points.push_back(Point3f(object->clouds[guesses[i].img_idx].points[m].x,
                                 object->clouds[guesses[i].img_idx].points[m].y,
                                 object->clouds[guesses[i].img_idx].points[m].z));
      }

      printf("Points size = %d\n", points.size());
      vector<Point2f> projectedPoints;
      projectPoints(Mat(points), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);
      guesses[i].projectedPoints = projectedPoints;
    }

    // filter overlappping guesses
    printf("Filtering overlapping poses\n");
    filterOverlappingPoses(imgPoints, guesses);
    int64 _t7 = cvGetTickCount();
    printf("filterOverlappingPoses: time elapsed %f\n", float(_t7 - _t6) / cvGetTickFrequency() * 1e-6);

    int64 _t8 = cvGetTickCount();
    nullCloudFilter(guesses, img, object, imgPoints);
    int64 _t9 = cvGetTickCount();
    printf("nullCloudFiltering: time elapsed %f\n", float(_t9 - _t8) / cvGetTickFrequency() * 1e-6);

    cout << "inliers size " << guesses.size() << endl;

    for (size_t i = 0; i < guesses.size(); i++)
    {
      vector<Point3f> points;
      for (size_t m = 0; m < object->clouds[guesses[i].img_idx].points.size(); m++)
      {
        points.push_back(Point3f(object->clouds[guesses[i].img_idx].points[m].x,
                                 object->clouds[guesses[i].img_idx].points[m].y,
                                 object->clouds[guesses[i].img_idx].points[m].z));
      }
      if (isPrintResults)
      {
        cout << "rvec: " << guesses[i].rvec.at<double> (0, 0) << " " << guesses[i].rvec.at<double> (1, 0) << " "
            << guesses[i].rvec.at<double> (2, 0) << endl;
        cout << "tvec: " << guesses[i].tvec.at<double> (0, 0) << " " << guesses[i].tvec.at<double> (1, 0) << " "
            << guesses[i].tvec.at<double> (2, 0) << endl;
      }
      vector<Point3f> rotated_object_points;
      //apply found R and T to point cloud
      project3dPoints(points, guesses[i].rvec, guesses[i].tvec, rotated_object_points);

      vector<Point2f> projectedPoints;
      projectPoints(Mat(points), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);
     //rotated_object_points - found pose of object

      //publish pose of object
      //publishPoints(rotated_object_points, points_pub, objectIndex);
      fullObjectPoints.insert(fullObjectPoints.end(), rotated_object_points.begin(), rotated_object_points.end());
      fullObjectIndices.insert(fullObjectIndices.end(), rotated_object_points.size(), objectIndex);

      drawProjectedPoints(rotated_object_points, objectIndex);

      // show inliers
      if (isDrawInliers)
      {
#ifndef CHESSBOARD
        cout << "Inliers count = " << guesses[i].inliers.size() << endl;
        drawInliers(img, objectIndex, guesses[i].img_idx, guesses[i].objPoints, guesses[i].imagePoints, guesses[i].inliers);
#else
        for (int imageIndex = 0; imageIndex < object->imagesNum; imageIndex++)
        {
          vector<int> inliers = guesses[i].inliers;
          vector<int>::iterator it;
          for (it = inliers.begin(); it != inliers.end();)
          {
            if (guesses[i].imageIndexes[*(it)] != imageIndex)
            {
              it = inliers.erase(it);
            }
            else
            {
              ++it;
            }
          }
          drawInliers(img, objectIndex, imageIndex, guesses[i].objPoints, guesses[i].imagePoints, inliers);
        }

#endif
      }

      ObjectInfo found_object;
      found_object.objectId = objectIndex;
      found_object.objectName = object->name;
      found_object.rvec = guesses[i].rvec;
      found_object.tvec = guesses[i].tvec;
      res_objects.push_back(found_object);
      printf("Object id %d, fine inliers %d\n", objectIndex, guesses[i].fineInliersCount);
    }
  }

  publishPoints(fullObjectPoints, points_pub, fullObjectIndices);

  printf("isDrawProjection = %d, isNode = %d", isDrawProjection, isNode);
  if (isDrawProjection && !isNode)
  {
    if (!isSaveImages)
    {
      namedWindow("projection");
      Mat smallProjImg;
      resize(projImg, smallProjImg, Size(), params.scale, params.scale);
      imshow("projection", smallProjImg);
      waitKey(0);
      waitKey(0);
    }
    else
    {
      string imageName = testName + "_projection.jpg";
      imwrite(imageName, projImg);
    }
  }
  if (isDrawCorrespondence)
    drawCorrespondence(img, keypoints, matches);
}

void TrainingSet::drawCorrespondencesOneTrainImage(const Mat& img1, const Mat& img2,
                                                   const vector<KeyPoint>& keypoints1,
                                                   const vector<KeyPoint>& keypoints2, const vector<int>& matches,
                                                   const vector<int>& inlierIndices)
{
  string winName = "correspondence";
  namedWindow(winName, CV_WINDOW_AUTOSIZE);

  Scalar RED = CV_RGB(255, 0, 0); // red keypoint - point without corresponding point
  Scalar GREEN = CV_RGB(0, 255, 0); // green keypoint - point having correct corresponding point
  Scalar BLUE = CV_RGB(0, 0, 255); // blue keypoint - point having incorrect corresponding point

  Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
  Mat drawImg(size, CV_MAKETYPE(img1.depth(), 3));
  Mat drawImg1 = drawImg(Rect(0, 0, img1.cols, img1.rows));
  cvtColor(img1, drawImg1, CV_GRAY2RGB);
  Mat drawImg2 = drawImg(Rect(img1.cols, 0, img2.cols, img2.rows));
  cvtColor(img2, drawImg2, CV_GRAY2RGB);

  // draw keypoints
  for (vector<KeyPoint>::const_iterator it = keypoints1.begin(); it < keypoints1.end(); ++it)
  {
    //circle(drawImg, it->pt, 3, RED);
    //circle(drawImg, it->pt, it->size/2, CV_RGB(0,200,200));
  }
  for (vector<KeyPoint>::const_iterator it = keypoints2.begin(); it < keypoints2.end(); ++it)
  {
    //Point p = it->pt;
    //circle(drawImg, Point2f(p.x+img1.cols, p.y), 3, RED);
    //circle(drawImg, Point2f(p.x+img1.cols, p.y), it->size/2, CV_RGB(0,200,200));
  }

  // draw matches
  for (size_t i = 0; i < matches.size(); i++)
  {
    Point2f testPoint = keypoints2[matches[i]].pt;
    Point2f objPoint = keypoints1[i].pt;
    Point2f testPoint2 = Point2f(std::min(testPoint.x + img1.cols, float(drawImg.cols - 1)), testPoint.y);
    circle(drawImg, objPoint, 3, GREEN);
    circle(drawImg, testPoint2, 3, GREEN);
    line(drawImg, objPoint, testPoint2, GREEN);
  }
  imshow(winName, drawImg);
  waitKey(0);
}

void TrainingSet::drawClusters(const cv::Mat img, const std::vector<cv::Point2f>& imagePoints,
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

  if (!isSaveImages)
  {
    Mat smallDrawImg;
    resize(drawImg, smallDrawImg, Size(), params.scale, params.scale);
    imshow(clusterWinName, smallDrawImg);
    waitKey(0);
  }
  else
  {
    string imageName = testName + "_clusters.jpg";
    imwrite(imageName, drawImg);
  }
}

void TrainingSet::drawInliers(const Mat img, int obj_ind, int img_ind, const vector<Point2f> objPoints, const vector<
    Point2f> imagePoints, const vector<int>& inliers, bool isCSN, int rect_ind, Rect rect)
{
  Scalar PINK = CV_RGB(255, 130, 230);
  Object* object = objects[obj_ind];

  const Mat* img1 = &img;

  if (!isCSN && object->images[img_ind].empty())
  {
    cout << "We don't load training images from disk now. It's help us to minimize required size of RAM" << endl;
    return;
  }

  const Mat* img2 = isCSN ? &object->csnImages[img_ind] : &object->images[img_ind];

  Size size(img1->cols + img2->cols, MAX(img1->rows, img2->rows));
  Mat drawImg(size, CV_MAKETYPE(img1->depth(), 3));
  drawImg.setTo(Scalar::all(0));
  Mat drawImg1 = drawImg(Rect(0, 0, img1->cols, img1->rows));
  cvtColor(*img1, drawImg1, CV_GRAY2RGB);
  Mat drawImg2 = drawImg(Rect(img1->cols, 0, img2->cols, img2->rows));
  cvtColor(*img2, drawImg2, CV_GRAY2RGB);
  int index;
  for (size_t i = 0; i < inliers.size(); i++)
  {
    index = inliers[i];
    circle(drawImg, imagePoints[index], 3, PINK);
    circle(drawImg, Point(objPoints[index].x + img.cols, objPoints[index].y), 3, PINK);
    line(drawImg, imagePoints[index], Point(objPoints[index].x + img1->cols, objPoints[index].y),
         CV_RGB(rand()%255, rand()%255, rand()%255));
  }
  if (rect_ind >= 0)
    rectangle(drawImg, rect, Scalar(0, 255, 0));

  stringstream objStream;
  objStream << obj_ind;

  stringstream imgStream;
  imgStream << img_ind;

  string winname = "inliers, object " + objStream.str();// , view " + imgStream.str();
  namedWindow(winname, CV_WINDOW_AUTOSIZE);
  Mat smallDrawImg;
  resize(drawImg, smallDrawImg, Size(), params.scale, params.scale);

#ifndef PAS_FEATURES  
  if (!isSaveImages)
  {
    imshow(winname, smallDrawImg);
    waitKey(0);
  }
  //  waitKey(0);
#else
  stringstream str;
  if(isCSN)
  str << "csn";
  str << "Inliers" << obj_ind << "_" << img_ind << "_" << rect_ind << ".png";
  imwrite(str.str(), smallDrawImg);
  if(isCSN)
  {
    //imshow(winname, smallDrawImg);
    //waitKey(1000);
  }
#endif
}

void TrainingSet::drawProjectedPoints(const Mat img, int obj_ind, int img_ind, const vector<Point3f> &object_points,
                                      const vector<Point2f> objPoints, const vector<Point2f> imagePoints,
                                      const Mat& camera_matrix, const Mat& dist_coeffs, const Mat &rvec,
                                      const Mat& tvec, bool isCSN, int rect_ind, Rect rect)
{
  if (object_points.empty())
    return;
  Scalar PINK = CV_RGB(255, 130, 230);
  Object* object = objects[obj_ind];

  const Mat* img1 = &img;
  const Mat* img2 = isCSN ? &object->csnImages[img_ind] : &object->images[img_ind];

  Size size(img1->cols + img2->cols, MAX(img1->rows, img2->rows));
  Mat drawImg(size, CV_MAKETYPE(img1->depth(), 3));
  drawImg.setTo(Scalar::all(0));
  Mat drawImg1 = drawImg(Rect(0, 0, img1->cols, img1->rows));
  cvtColor(*img1, drawImg1, CV_GRAY2RGB);
  Mat drawImg2 = drawImg(Rect(img1->cols, 0, img2->cols, img2->rows));
  cvtColor(*img2, drawImg2, CV_GRAY2RGB);

  vector<Point2f> projected_points;
  projected_points.resize(object_points.size());
  projectPoints(Mat(object_points), rvec, tvec, camera_matrix, dist_coeffs, projected_points);

  RNG rng(cvGetTickCount());
  for (size_t i = 0; i < projected_points.size(); i++)
  {
    int min = 50;
    int max = 255;
    Scalar color = Scalar(rng.uniform(min, max), rng.uniform(min, max), rng.uniform(min, max));
    circle(drawImg, projected_points[i], 5, color);
    circle(drawImg, imagePoints[i], 1, Scalar(0, 0, 255));
    circle(drawImg, Point(objPoints[i].x + img.cols, objPoints[i].y), 5, color);
  }
  for (size_t i = 0; i < object->keypoints[img_ind].size(); i++)
  {
    circle(drawImg, object->keypoints[img_ind][i].pt + Point2f(img.cols, 0), 1, Scalar(0, 0, 255));
  }
  if (rect_ind >= 0)
    rectangle(drawImg, rect, Scalar(0, 255, 0));

  string winname = "transform";
  namedWindow(winname, CV_WINDOW_AUTOSIZE);
  Mat smallDrawImg;
  resize(drawImg, smallDrawImg, Size(), params.scale, params.scale);

#ifndef PAS_FEATURES
  imshow(winname, smallDrawImg);
  waitKey(0);
#else
  stringstream str;
  if(isCSN)
  str << "csn";
  str << "Transform" << obj_ind << "_" << img_ind << "_" << rect_ind << ".png";
  imwrite(str.str(), smallDrawImg);
  if(isCSN)
  {
    //imshow(winname, smallDrawImg);
    //waitKey(1000);
  }
#endif
}

void TrainingSet::drawCorrespondence(const Mat& img, const vector<KeyPoint>& keypoints, const vector<DMatch>& matches)
{
  string winName = "correspondence";
  namedWindow(winName, CV_WINDOW_AUTOSIZE);
  Scalar RED = CV_RGB(255, 0, 0);
  Scalar BLUE = CV_RGB(0, 0, 255);

  for (int objInd = 0; objInd < objNum; objInd++)
  {
    Object* obj = objects[objInd];
    for (int imgInd = 0; imgInd < obj->imagesNum; imgInd++)
    {
      if (obj->images[imgInd].empty())
      {
        cout << "We don't load training images from disk now. It's help us to minimize required size of RAM" << endl;
        return;
      }
      Size size(img.cols + obj->images[imgInd].cols, MAX(img.rows, obj->images[imgInd].rows));
      Mat drawImg(size, CV_MAKETYPE(img.depth(), 3));
      drawImg.setTo(Scalar::all(0));
      Mat drawImg1 = drawImg(Rect(0, 0, img.cols, img.rows));
      cvtColor(img, drawImg1, CV_GRAY2RGB);
      Mat drawImg2 = drawImg(Rect(img.cols, 0, obj->images[imgInd].cols, obj->images[imgInd].rows));
      cvtColor(obj->images[imgInd], drawImg2, CV_GRAY2RGB);

      for (vector<KeyPoint>::const_iterator it = keypoints.begin(); it < keypoints.end(); ++it)
      {
        circle(drawImg, it->pt, 3, RED);
      }

      for (vector<KeyPoint>::const_iterator it = obj->keypoints[imgInd].begin(); it < obj->keypoints[imgInd].end(); ++it)
      {
        Point p = it->pt;
        circle(drawImg, Point(p.x + img.cols, p.y), 3, RED);
      }

      vector<Point2f> srcpts, dstpts;
      for (vector<DMatch>::const_iterator it = matches.begin(); it < matches.end(); ++it)
      {
        if (obj_indices[it->trainIdx] == objInd && img_indices[it->trainIdx] == imgInd)
        {
          Point pt1 = keypoints[it->queryIdx].pt, pt2 = obj->keypoints[imgInd][keypoint_indices[it->trainIdx]].pt;
          circle(drawImg, pt1, 3, BLUE);
          circle(drawImg, Point(pt2.x + img.cols, pt2.y), 3, BLUE);
          line(drawImg, pt1, Point(pt2.x + img.cols, pt2.y), BLUE);
        }
      }

      Mat smallDrawImg;
      resize(drawImg, smallDrawImg, Size(), params.scale, params.scale);
      imshow(winName, smallDrawImg);
      waitKey(0);
    }
  }
}

void TrainingSet::setTestCameraInfo(const image_geometry::PinholeCameraModel& cam_model)
{
  cam_model.intrinsicMatrix().copyTo(cameraMatrix);
  cam_model.distortionCoeffs().copyTo(distCoeffs);
}
