#include "textured_object_detection/object.h"
#include "opencv/highgui.h"
#include <fstream>

using namespace std;
using namespace cv;

//Find nearest point wrt Chebyshev distance (chessboard distance)
Point findNearestPoint( Mat &mask, Point pt )
{
    int d=1;
    int x0 = pt.x;
    int y0 = pt.y;
    int maxDistance = std::max( mask.rows, mask.cols );
    while( d < maxDistance )
    {
        for( int x=x0-d;x<=x0+d;x++ )
        {
            if( mask.at<uint16_t>( x, y0-d ) )
            {
                return Point( x, y0-d );
            }
            if( mask.at<uint16_t>( x, y0+d ) )
            {
                return Point( x, y0+d );
            }
        }

        for( int y=y0-d;y<=y0+d;y++ )
        {
            if( mask.at<uint16_t>( x0-d, y ) )
            {
                return Point( x0-d, y );
            }
            if( mask.at<uint16_t>( x0+d, y ) )
            {
                return Point( x0+d, y );
            }
        }
        d++;
    }
    //mask is zero at every point
    assert(false);
}

void Object::filterPoints( const Mat &image, vector<KeyPoint > &points, Mat &mask, bool filterBoundaryPoints, bool replaceByNearestPoint)
{
  vector<KeyPoint>::iterator it;
  for (it = points.begin(); it != points.end();)
  {
    if (mask.at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) == 0)
    {
        if( replaceByNearestPoint )
        {
            Point nearest = findNearestPoint( mask, Point( (int)(*it).pt.y, (int)(*it).pt.x ) );
            mask.at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) = mask.at<uint16_t>( nearest.x, nearest.y );
        }
        else
        {
            it = points.erase(it);
        }
    }
    else
    {
      ++it;
    }
  }

  if( filterBoundaryPoints )
  {
    for(vector<KeyPoint>::iterator it = points.begin(); it != points.end(); )
    {
        int diff = it->size/2 + 1;
        int x = std::max(0, (int)(it->pt.x - diff));
        int y = std::max(0, (int)(it->pt.y - diff));
        Mat keypointMask = mask(cv::Rect(x, y, it->size, it->size));
        if (countNonZero(keypointMask) < it->size*it->size)
        {
            it = points.erase(it);
        }
        else
        {
           ++it;
        }
    }
  }
}

void Object::filterKeyPoints(int img_ind, Mat& mask, bool replaceByNearestPoint)
{
  filterPoints( images[img_ind], keypoints[img_ind], mask, true, replaceByNearestPoint );
}

void Object::setPointIndices(int img_ind, const image_geometry::PinholeCameraModel* camModel, Mat& mask)
{
  mask.create(images[img_ind].rows, images[img_ind].cols, CV_16UC1);
  mask.setTo(Scalar::all(0));
  for (size_t i = 0; i < clouds[img_ind].points.size(); ++i)
  {
    cv::Point3d pt_cv(clouds[img_ind].points[i].x, clouds[img_ind].points[i].y, clouds[img_ind].points[i].z);
    cv::Point2d uv;
    camModel->project3dToPixel(pt_cv, uv);
    int x = (int)uv.x;
    int y = (int)uv.y;
    if (x > 0 && x < images[img_ind].cols && y > 0 && y < images[img_ind].rows)
    {
      for (int xx = x - 10; xx < x + 10 && xx > 0 && xx < images[img_ind].cols; xx++)
        for (int yy = y - 10; yy < y + 10 && yy > 0 && yy < images[img_ind].rows; yy++)
          mask.at<uint16_t> (yy, xx) = i + 1;
    }
  }
}

void Object::computeDescriptors(int img_ind, Mat& mask, const cv::FeatureDetector* detector,
                                const cv::DescriptorExtractor* descriptor)
{
  detector->detect(images[img_ind], keypoints[img_ind]);
  vector<KeyPoint>::iterator it;
  bool replaceByNearestPoint = false;
#ifdef PAS_FEATURES
  replaceByNearestPoint = true;
#endif
  filterKeyPoints(img_ind, mask, replaceByNearestPoint);
  for (vector<KeyPoint>::const_iterator it = keypoints[img_ind].begin(); it != keypoints[img_ind].end(); it++)
    pointsIndices[img_ind].push_back(mask.at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) - 1);

  descriptor->compute(images[img_ind], keypoints[img_ind], descriptors[img_ind]);
}

bool isFileExist(const string& filename)
{
  ifstream file;
  file.open(filename.c_str());

  bool result = file.is_open();
  file.close();

  return result;
}

void saveDescriptors(const string& filename, const Mat& descriptors)
{
  FileStorage fs( filename.c_str(), FileStorage::WRITE );
  fs << "descriptor" << descriptors;
  fs.release();
}

void loadDescriptors(const string& filename, Mat& descriptors)
{
  FileStorage fs;
  fs.open(filename.c_str(), FileStorage::READ);
  fs["descriptor"] >> descriptors;
  fs.release();
}

void saveKeypoints(const string& filename, const vector<KeyPoint>& keypoints)
{
  ofstream keypointsFile(filename.c_str());
  keypointsFile << keypoints.size() << endl;
  for (vector<KeyPoint>::const_iterator it = keypoints.begin(); it != keypoints.end(); it++)
    keypointsFile << (int)(*it).pt.y << " " << (int)(*it).pt.x << endl;
  keypointsFile.close();
}

void loadKeypoints(const string& filename, vector<KeyPoint>& keypoints)
{
  keypoints.clear();

  size_t size;

  ifstream kptFile;
  kptFile.open(filename.c_str());
  kptFile >> size;
  keypoints.resize(size);
  for (size_t ind = 0; ind < size; ind++)
    kptFile >> keypoints[ind].pt.y >> keypoints[ind].pt.x;
  kptFile.close();
}

void savePointsIndices(const string& filename, const vector<uint16_t>& indices)
{
  ofstream piWriteFile(filename.c_str());
  piWriteFile << indices.size() << endl;
  for (vector<uint16_t>::const_iterator it = indices.begin(); it != indices.end(); it++)
    piWriteFile << *it << endl;
  piWriteFile.close();
}

void loadPointsIndices(const string& filename, vector<uint16_t>& indices)
{
  indices.clear();

  size_t size;

  ifstream piFile;
  piFile.open(filename.c_str());
  piFile >> size;
  indices.resize(size);
  for (size_t ind = 0; ind < size; ind++)
    piFile >> indices[ind];
  piFile.close();
}


void Object::addImage(const Mat& img, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                      const image_geometry::PinholeCameraModel* cam_model,
                      const cv::FeatureDetector* detector, const cv::DescriptorExtractor* descriptor,
                      const string& path, const string& index, bool isRecalcDescriptors)
{
  imagesNum++;
  images.resize(imagesNum);
  clouds.resize(imagesNum);
  descriptors.resize(imagesNum);
  pointsIndices.resize(imagesNum);
  keypoints.resize(imagesNum);
  clouds[imagesNum - 1] = cloud;

#ifdef CHESSBOARD
  rvecs.resize(imagesNum);
  tvecs.resize(imagesNum);
  if (imagesNum == 1)
  {
    Mat rvec1, tvec1;
    rvec1.create(3, 1, CV_64FC1);
    tvec1.create(3, 1, CV_64FC1);
    rvec1.at<double> (0, 0) = rvec1.at<double> (1, 0) = rvec1.at<double> (2, 0) = 0.0;
    tvec1.at<double> (0, 0) = tvec1.at<double> (1, 0) = tvec1.at<double> (2, 0) = 0.0;
    rvec1.copyTo(rvecs[0]);
    tvec1.copyTo(tvecs[0]);
  }
  else
  {
    FileStorage fs;
    string chessPath = path + "chess" + index + ".xml";
    fs.open(chessPath.c_str(), FileStorage::READ);
    fs["rvec"] >> rvecs[imagesNum - 1];
    fs["tvec"] >> tvecs[imagesNum - 1];
    fs.release();
  }
#endif

  string descsPath = path + index + ".xml";
  bool isPreparedFilesExist = isFileExist(descsPath);

  std::string kptPath = path + index  + ".kpt";
  isPreparedFilesExist = isPreparedFilesExist && isFileExist(kptPath);

  std::string piPath = path + index  + ".pi";
  isPreparedFilesExist = isPreparedFilesExist && isFileExist(piPath);

  int img_ind = imagesNum - 1;

#ifdef CHAMFER_PLUS_TEXTURE
  edgels.resize(imagesNum);
  edgelsIndices.resize(imagesNum);
  edgelsMaskImages.resize(imagesNum);
#endif

  if (!isPreparedFilesExist || isRecalcDescriptors)
  {
    img.copyTo(images[imagesNum - 1]);

    Mat mask;
    setPointIndices(img_ind, cam_model, mask);
    computeDescriptors(img_ind, mask, detector, descriptor);

    saveDescriptors(descsPath, descriptors[img_ind]);
    saveKeypoints(kptPath, keypoints[img_ind]);
    savePointsIndices(piPath, pointsIndices[img_ind]);
  }
  else
  {
    loadDescriptors(descsPath, descriptors[img_ind]);
    loadKeypoints(kptPath, keypoints[img_ind]);
    loadPointsIndices(piPath, pointsIndices[img_ind]);
  }

#ifdef CHAMFER_PLUS_TEXTURE
  edgelsMaskImages[img_ind] = maskImages[img_ind].clone();

  Mat edges;
  const double cannyThreshold1 = GET_CANNY_THRESHOLD_1();
  const double cannyThreshold2 = GET_CANNY_THRESHOLD_2();

  Canny( images[img_ind], edges, cannyThreshold1, cannyThreshold2 );
  for( int i=0;i<edges.rows;i++ )
  {
    for( int j=0;j<edges.cols;j++ )
    {
      if( edges.at<uchar>(i, j) != 0 )
      {
        KeyPoint keypoint( j, i, 0 );
        edgels[img_ind].push_back( keypoint );
      }
    }
  }
  filterPoints( images[img_ind], edgels[img_ind], edgelsMaskImages[img_ind], false, true );
  for (vector<KeyPoint>::const_iterator it = edgels[img_ind].begin(); it != edgels[img_ind].end(); it++)
    edgelsIndices[img_ind].push_back(edgelsMaskImages[img_ind].at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) - 1);
#endif

#ifdef PAS_FEATURES
  csns.resize(imagesNum);
  csnImages.resize(imagesNum);
  ContourSegmentNetwork csn;
  csn.calcContourSegmentNetwork(img, true);
  csns[imagesNum - 1] = csn;
  csnImages[imagesNum - 1] = csn.drawAllSegments();
#endif
}

Object::Object(const std::string _name, const std::string _images_path, int _images_num,
               const image_geometry::PinholeCameraModel* _cam_model, const cv::FeatureDetector* detector,
               const cv::DescriptorExtractor* descriptor, bool isRecalcDescriptors) : cam_model(*_cam_model)
{
  name = _name;
  imagesNum = 0;

  for (int k = 0; k < _images_num; k++)
  {
    stringstream out;
    out << (k + 1);
    string path = _images_path + out.str() + ".png";

    Mat img = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    path = _images_path + out.str() + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;

    FILE *file = fopen(path.c_str(), "r");
    if (file)
    {
      fclose(file);
      sensor_msgs::PointCloud2 cloud_blob;
      pcl::io::loadPCDFile(path.c_str(), cloud_blob);
      pcl::fromROSMsg(cloud_blob, cloud);
    }
    else
    {
      ifstream txtfile;
      path = _images_path + out.str() + ".txt";
      txtfile.open(path.c_str());
      vector<Point3f> points;
      while (!txtfile.eof())
      {
        pcl::PointXYZ p;
        txtfile >> p.x >> p.y >> p.z;
        cloud.points.push_back(p);
        points.push_back(Point3f(p.x, p.y, p.z));
      }
      if (!k)
      {
        stddev = computeStddev(points);
      }
      txtfile.close();
    }

    path = _images_path;

    addImage(img, cloud, _cam_model, detector, descriptor, path, out.str(), isRecalcDescriptors);
  }
}

Object::~Object()
{
}

struct Point3fDist
{
   double distance;
   Point3f point;

   Point3fDist()
   {
     distance = 0.f;
     point = Point3f();
   }

   Point3fDist(double _distance, Point3f _point)
   {
       distance = _distance;
       point = _point;
   }
   static bool comparison(const Point3fDist& a, const Point3fDist& b)
   {
       return a.distance < b.distance;
   }
};


float computeStddevWithFilter(const std::vector<Point3f>& points)
{
  Point3f center(0, 0, 0);
  for(size_t j = 0; j < points.size(); j++)
  {
    center += points[j];
  }

  center = center*(1.f/points.size());

  vector<Point3fDist> pointsBuffer;
  pointsBuffer.resize(points.size());

  for(size_t j = 0; j < pointsBuffer.size(); j++)
  {
    pointsBuffer[j] = Point3fDist(norm(points[j] - center), points[j]);
  }

  sort(pointsBuffer.begin(), pointsBuffer.end(), Point3fDist::comparison);

  vector<Point3f> pointsCopy;
  pointsCopy.resize((size_t)(points.size()*0.9));
  for(size_t j = 0; j < pointsCopy.size(); j++)
  {
    pointsCopy[j] = pointsBuffer[j].point;
  }

  float stddev = computeStddev(pointsCopy);
  return stddev;
}

float computeStddev(const std::vector<Point3f>& points)
{
  Point3f sum(0, 0, 0);
  float sum2 = 0;
  for(size_t j = 0; j < points.size(); j++)
  {
    sum += points[j];
    sum2 += points[j].dot(points[j]);
  }
  float stddev = sqrt(sum2/points.size() - sum.dot(sum)/(points.size()*points.size()));

  return stddev;
}

float calcQuantileValue(vector<float> dists, float quantile)
{
  sort(dists.begin(), dists.end());
  return dists[int(floor(dists.size()*quantile))];
}

Point3f calcMean(const vector<Point3f>& points)
{
  Point3f mean;
  for(size_t i = 0; i < points.size(); i++)
  {
    mean += points[i];
  }

  mean = mean*(1.0/points.size());
  return mean;
}

void filterOutlierPoints(vector<Point3f>& points, float quantile)
{
  Point3f mean = calcMean(points);
  vector<float> dists;
  for(size_t i = 0; i < points.size(); i++)
  {
    dists.push_back(norm(points[i] - mean));
  }
  float quantileValue = calcQuantileValue(dists, 0.9);

  vector<Point3f> filtered;
  for(size_t i = 0; i < points.size(); i++)
  {
    if(norm(points[i] - mean) < quantileValue)
    {
      filtered.push_back(points[i]);
    }
  }

  points = filtered;
}
