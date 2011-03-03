#include <ftd/discretizers/surface_normal_discretizer.h>

#include "ftd/pcl_processing.h"

#include <opencv/cv.h>
#include <pcl/io/pcd_io.h>
#include <ftd/pcl_processing.h>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>


// ----------------------------------------------------------------------------
ftd::discretizers::
SurfaceNormalDiscretizer::
SurfaceNormalDiscretizer ()
{

}


// ----------------------------------------------------------------------------
ftd::discretizers::
SurfaceNormalDiscretizer::
~SurfaceNormalDiscretizer ()
{

}


// ----------------------------------------------------------------------------
void
ftd::discretizers::
SurfaceNormalDiscretizer::
discretize (
  const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
  const int regionWidth,
  const int regionHeight,
  unsigned char * discretizedData )
{
  //ComputeNormals cn_;
  ComputeNormalsUnorganized cn_;
  
  std::cerr << "Received a PointCloud2 with " << cloud2->width * cloud2->height << " points." << std::endl;
  cn_.setInputCloud (cloud2);
  //cn_.setKSearch (50);
  cn_.compute ();

  pcl::PointCloud<pcl::Normal> normals;
  cn_.getOutput (normals);
  

  const int numOfCharsPerElement = 1;
  
  IplImage * discretizedAngles = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  //IplImage * discretizedOrientations = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  cvSetZero (discretizedAngles);
  for (int rowIndex = 0; rowIndex < cloud2->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < cloud2->width; ++colIndex)
    {
      if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0]))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      
      const float viewPointAngle = acos (-normals.points[rowIndex*cloud2->width+colIndex].normal[2]);

      const float projectedX = normals.points[rowIndex*cloud2->width+colIndex].normal[0];
      const float projectedY = normals.points[rowIndex*cloud2->width+colIndex].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }
  //std::cerr << std::endl;
  


  const int imageWidth = discretizedAngles->width;
  const int imageHeight = discretizedAngles->height;
  
  
  // discretize  
  const int horizontalSamples = imageWidth/regionWidth;
  const int verticalSamples = imageHeight/regionHeight;
  
  memset (discretizedData, 0, horizontalSamples*verticalSamples*sizeof (unsigned char));
    
  for (int rowIndex = 0; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples; ++colIndex)
    {
      unsigned char values = 0;
      
      for (int subRowIndex = 0; subRowIndex < regionHeight; ++subRowIndex)
      {
        for (int subColIndex = 0; subColIndex < regionWidth; ++subColIndex)
        {
          const int binIndex = static_cast<int>(CV_IMAGE_ELEM(discretizedAngles, float, rowIndex*regionHeight+subRowIndex, colIndex*regionWidth+subColIndex)*8.0f);
          
          if (binIndex <= 7)
          {
            values |= 1 << binIndex;
          }
        }
      }
      
      discretizedData[rowIndex*horizontalSamples+colIndex] = values;

      //if (values == 0)
      //{
      //  mask[rowIndex*horizontalSamples+colIndex] = 0;
      //}
    }
  }
  
  cvReleaseImage (&discretizedAngles);
}


// ----------------------------------------------------------------------------
void
ftd::discretizers::
SurfaceNormalDiscretizer::
discretize (
  const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
  const ::stereo_msgs::DisparityImageConstPtr & disparityMsg,
  const int regionWidth,
  const int regionHeight,
  std::vector<unsigned char> & discretizedData )
{
        // get disparity image
    CvMat * disparityMatrix = cvCreateMatHeader (disparityMsg->image.height, disparityMsg->image.width, CV_32FC1);
    cvSetData (disparityMatrix, static_cast<void*> (const_cast<unsigned char*> (&(disparityMsg->image.data[0]))), disparityMsg->image.step);

    ROS_INFO("Created disparity matrix"); 
    
    IplImage * disparityImage = cvCreateImage (cvSize (disparityMatrix->cols, disparityMatrix->rows), IPL_DEPTH_32F, 1);
    cvSetZero (disparityImage);

    ROS_INFO("Created disparity image"); 

    for (int rowIndex = 0; rowIndex < disparityImage->height; ++rowIndex)
    {
      for (int colIndex = 0; colIndex < disparityImage->width; ++colIndex)
      {
        const float dispValue = CV_MAT_ELEM (*disparityMatrix, float, rowIndex, colIndex);
        if (dispValue > 90.0f /*&& dispValue < 120.0f*/)
        {
          CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) = dispValue;//(dispValue-90.0f)*20.0f;
        }
      }
    }
    
    ROS_INFO ("Filled disparity image");       


  //ComputeNormals cn_;
  ComputeNormalsUnorganized cn_;
  
  std::cerr << "Received a PointCloud2 with " << cloud2->width * cloud2->height << " points." << std::endl;
  cn_.setInputCloud (cloud2);
  //cn_.setKSearch (50);
  cn_.compute ();

  pcl::PointCloud<pcl::Normal> normals;
  cn_.getOutput (normals);
  

  const int numOfCharsPerElement = 1;
  
  IplImage * discretizedAngles = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  //IplImage * discretizedOrientations = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  cvSetZero (discretizedAngles);
  for (int rowIndex = 0; rowIndex < cloud2->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < cloud2->width; ++colIndex)
    {
      if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0])
        || (CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) < 90.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      
      const float viewPointAngle = acos (-normals.points[rowIndex*cloud2->width+colIndex].normal[2]);

      const float projectedX = normals.points[rowIndex*cloud2->width+colIndex].normal[0];
      const float projectedY = normals.points[rowIndex*cloud2->width+colIndex].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }
  //std::cerr << std::endl;
  


  const int imageWidth = discretizedAngles->width;
  const int imageHeight = discretizedAngles->height;
  
  
  // discretize  
  const int horizontalSamples = imageWidth/regionWidth;
  const int verticalSamples = imageHeight/regionHeight;
  
  for (std::vector<unsigned char>::iterator iter = discretizedData.begin (); iter != discretizedData.end (); ++iter)
  {
    *iter = 0;
  }
    
  for (int rowIndex = 0; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples; ++colIndex)
    {
      unsigned char values = 0;
      
      for (int subRowIndex = 0; subRowIndex < regionHeight; ++subRowIndex)
      {
        for (int subColIndex = 0; subColIndex < regionWidth; ++subColIndex)
        {
          const int binIndex = static_cast<int>(CV_IMAGE_ELEM(discretizedAngles, float, rowIndex*regionHeight+subRowIndex, colIndex*regionWidth+subColIndex)*8.0f);
          
          if (binIndex <= 7)
          {
            values |= 1 << binIndex;
          }
        }
      }
      
      discretizedData[rowIndex*horizontalSamples+colIndex] = values;

      //if (values == 0)
      //{
      //  mask[rowIndex*horizontalSamples+colIndex] = 0;
      //}
    }
  }
  
  cvReleaseImage (&discretizedAngles);
  cvReleaseImage (&disparityImage);
}


// ----------------------------------------------------------------------------
void
ftd::discretizers::
SurfaceNormalDiscretizer::
discretize (
  const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
  const int regionWidth,
  const int regionHeight,
  std::vector<unsigned char> & discretizedData )
{
  ComputeNormals cn_;
  //ComputeNormalsUnorganized cn_;
  
  std::cerr << "Received a PointCloud2 with " << cloud2->width * cloud2->height << " points." << std::endl;
  cn_.setInputCloud (cloud2);
  cn_.setKSearch (50);
  cn_.compute ();

  pcl::PointCloud<pcl::Normal> normals;
  cn_.getOutput (normals);
  

  const int numOfCharsPerElement = 1;
  
  IplImage * discretizedAngles = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  //IplImage * discretizedOrientations = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  cvSetZero (discretizedAngles);
  for (int rowIndex = 0; rowIndex < cloud2->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < cloud2->width; ++colIndex)
    {
      if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0]))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      
      const float viewPointAngle = acos (-normals.points[rowIndex*cloud2->width+colIndex].normal[2]);

      const float projectedX = normals.points[rowIndex*cloud2->width+colIndex].normal[0];
      const float projectedY = normals.points[rowIndex*cloud2->width+colIndex].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }
  //std::cerr << std::endl;
  


  const int imageWidth = discretizedAngles->width;
  const int imageHeight = discretizedAngles->height;
  
  
  for (std::vector<unsigned char>::iterator iter = discretizedData.begin (); iter != discretizedData.end (); ++iter)
  {
    *iter = 0;
  }
    
  
  // discretize
  const int horizontalSamples = imageWidth/regionWidth;
  const int verticalSamples = imageHeight/regionHeight;
  
    
  for (int rowIndex = 0; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples; ++colIndex)
    {
      unsigned char values = 0;
      
      for (int subRowIndex = 0; subRowIndex < regionHeight; ++subRowIndex)
      {
        for (int subColIndex = 0; subColIndex < regionWidth; ++subColIndex)
        {
          const int binIndex = static_cast<int>(CV_IMAGE_ELEM(discretizedAngles, float, rowIndex*regionHeight+subRowIndex, colIndex*regionWidth+subColIndex)*8.0f);
          
          if (binIndex <= 7)
          {
            values |= 1 << binIndex;
          }
        }
      }
      
      discretizedData[rowIndex*horizontalSamples+colIndex] = values;

      //if (values == 0)
      //{
      //  mask[rowIndex*horizontalSamples+colIndex] = 0;
      //}
    }
  }
  
  cvReleaseImage (&discretizedAngles);
}


// ----------------------------------------------------------------------------
void
ftd::discretizers::
SurfaceNormalDiscretizer::
discretize (
  const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
  IplImage * mask,
  const int regionWidth,
  const int regionHeight,
  std::vector<unsigned char> & discretizedData )
{
  //ComputeNormals cn_;
  ComputeNormalsUnorganized cn_;
  
  std::cerr << "Received a PointCloud2 with " << cloud2->width * cloud2->height << " points." << std::endl;
  cn_.setInputCloud (cloud2);
  //cn_.setKSearch (50);
  cn_.compute ();

  pcl::PointCloud<pcl::Normal> normals;
  cn_.getOutput (normals);
  

  const int numOfCharsPerElement = 1;
  
  IplImage * discretizedAngles = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  //IplImage * discretizedOrientations = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  cvSetZero (discretizedAngles);
  for (int rowIndex = 0; rowIndex < cloud2->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < cloud2->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (mask, unsigned char, rowIndex, colIndex) == 0)
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0]))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      
      std::cerr << ".";
      
      const float viewPointAngle = acos (-normals.points[rowIndex*cloud2->width+colIndex].normal[2]);

      const float projectedX = normals.points[rowIndex*cloud2->width+colIndex].normal[0];
      const float projectedY = normals.points[rowIndex*cloud2->width+colIndex].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }
  //std::cerr << std::endl;
  
  cvShowImage ("discretizedAngles", discretizedAngles);
  cvWaitKey (10);
  


  const int imageWidth = discretizedAngles->width;
  const int imageHeight = discretizedAngles->height;
  
  
  for (std::vector<unsigned char>::iterator iter = discretizedData.begin (); iter != discretizedData.end (); ++iter)
  {
    *iter = 0;
  }
    
  
  // discretize
  const int horizontalSamples = imageWidth/regionWidth;
  const int verticalSamples = imageHeight/regionHeight;
  
    
  for (int rowIndex = 0; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples; ++colIndex)
    {
      unsigned char values = 0;
      
      for (int subRowIndex = 0; subRowIndex < regionHeight; ++subRowIndex)
      {
        for (int subColIndex = 0; subColIndex < regionWidth; ++subColIndex)
        {
          const int binIndex = static_cast<int>(CV_IMAGE_ELEM(discretizedAngles, float, rowIndex*regionHeight+subRowIndex, colIndex*regionWidth+subColIndex)*8.0f);
          
          if (binIndex <= 7)
          {
            values |= 1 << binIndex;
            std::cerr << "_";
          }
        }
      }
      
      discretizedData[rowIndex*horizontalSamples+colIndex] = values;

      //if (values == 0)
      //{
      //  mask[rowIndex*horizontalSamples+colIndex] = 0;
      //}
    }
  }
  
  cvReleaseImage (&discretizedAngles);
}


// ----------------------------------------------------------------------------
void
ftd::discretizers::
SurfaceNormalDiscretizer::
discretizeNonDense (
  const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
  IplImage * mask,
  const int regionWidth,
  const int regionHeight,
  const int imageWidth,
  const int imageHeight,
  const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
  std::vector<unsigned char> & discretizedData )
{
  //ComputeNormals cn_;
  ComputeNormalsUnorganized cn_;
  
  std::cerr << "Received a PointCloud2 with " << cloud2->width * cloud2->height << " points." << std::endl;
  cn_.setInputCloud (cloud2);
  //cn_.setKSearch (50);
  cn_.compute ();

  pcl::PointCloud<pcl::Normal> normals;
  cn_.getOutput (normals);
  
  ::pcl::PointCloud< ::pcl::PointXYZ > cloud;
  //point_cloud::fromMsg (*cloud2, cloud);
  pcl::fromROSMsg(*cloud2, cloud);
  
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo (camInfoMsg);
  
  //std::cerr << "disc1" << std::endl;

  Eigen3::MatrixXf points_3d = Eigen3::MatrixXf::Ones (4, cloud.points.size ());
  for (unsigned int pointIndex = 0; pointIndex < points_3d.cols (); ++pointIndex)
  {
    points_3d (0, pointIndex) = cloud.points[pointIndex].x;
    points_3d (1, pointIndex) = cloud.points[pointIndex].y;
    points_3d (2, pointIndex) = cloud.points[pointIndex].z;
  }

  //std::cerr << "disc2" << std::endl;
  Eigen3::MatrixXf camera_pose = Eigen3::MatrixXf::Identity (3, 4);
  Eigen3::MatrixXf intrinsic_mat = Eigen3::MatrixXf::Identity (3, 3);
  intrinsic_mat (0, 0) = camera_model.fx ();
  intrinsic_mat (0, 2) = camera_model.cx ();
  intrinsic_mat (1, 1) = camera_model.fy ();
  intrinsic_mat (1, 2) = camera_model.cy ();

  //std::cerr << "disc3" << std::endl;
  Eigen3::MatrixXf points_2d = intrinsic_mat * camera_pose * points_3d;
  for (int i = 0; i < points_2d.cols (); ++i)
    points_2d.col (i) /= points_2d (2, i);
  
  const int numOfCharsPerElement = 1;
  
  std::cerr << "disc4" << std::endl;
  IplImage * discretizedAngles = cvCreateImage (cvSize (imageWidth, imageHeight), IPL_DEPTH_32F, 1);
  //IplImage * discretizedOrientations = cvCreateImage (cvSize (cloud2->width, cloud2->height), IPL_DEPTH_32F, 1);
  cvSetZero (discretizedAngles);
  
  for (int rowIndex = 0; rowIndex < imageHeight; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < imageWidth; ++colIndex)
    {
      CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
    }
  }
  
  //std::cerr << "disc5" << std::endl;
  for (int i = 0; i < points_2d.cols (); ++i)
  {
    const int colIndex = static_cast<int> (points_2d (0, i));
    const int rowIndex = static_cast<int> (points_2d (1, i));
    
    //std::cerr << "(" << colIndex << "," << rowIndex << ")";
    //if (px >= 0 && px < mask->width && py >= 0 && py < mask->height)
    {
      if (CV_IMAGE_ELEM (mask, unsigned char, rowIndex, colIndex) == 0)
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      //if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0]))
      //{
      //  CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
      //  continue;
      //}
      
      //std::cerr << ".";
      
      const float viewPointAngle = acos (-normals.points[i].normal[2]);

      const float projectedX = normals.points[i].normal[0];
      const float projectedY = normals.points[i].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }
  
  //std::cerr << "disc6" << std::endl;

  /*for (int rowIndex = 0; rowIndex < cloud2->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < cloud2->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (mask, unsigned char, rowIndex, colIndex) == 0)
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      if (isnan (normals.points[rowIndex*cloud2->width+colIndex].normal[0]))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 8.0f/8.0f;
        //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = 8.0f/8.0f;
        continue;
      }
      
      std::cerr << ".";
      
      const float viewPointAngle = acos (-normals.points[rowIndex*cloud2->width+colIndex].normal[2]);

      const float projectedX = normals.points[rowIndex*cloud2->width+colIndex].normal[0];
      const float projectedY = normals.points[rowIndex*cloud2->width+colIndex].normal[1];
      const float length = sqrt (projectedX*projectedX + projectedY*projectedY);
      
      const float orientationAngle = acos (projectedX/length);
      
      //const int viewPointAngleBinIndex = static_cast<int> (viewPointAngle/7);
      //const int orientationAngleBinIndex = static_cast<int> (orientationAngle/7);
      
      //CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = viewPointAngleBinIndex;
      //CV_IMAGE_ELEM (discretizedOrientations, float, rowIndex, colIndex) = orientationAngleBinIndex;
      
      if (viewPointAngle < (15.0f*3.14f/180.0f))
      {
        CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 1.0f/8.0f;
      }
      else if (viewPointAngle < (45.0f*3.14f/180.0f))
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 2.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 3.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 4.0f/8.0f;
          }
        }
      }
      else
      {
        if (orientationAngle < (30.0f*3.14f/180.0f))
        {
          CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 5.0f/8.0f;
        }
        else
        {
          if (projectedY < 0)
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 6.0f/8.0f;
          }
          else
          {
            CV_IMAGE_ELEM (discretizedAngles, float, rowIndex, colIndex) = 7.0f/8.0f;
          }
        }
      }

    }
  }*/
  //std::cerr << std::endl;
  
  cvShowImage ("discretizedAngles", discretizedAngles);
  cvWaitKey (10);
  


  //const int imageWidth = discretizedAngles->width;
  //const int imageHeight = discretizedAngles->height;
  
  
  for (std::vector<unsigned char>::iterator iter = discretizedData.begin (); iter != discretizedData.end (); ++iter)
  {
    *iter = 0;
  }
    
  
  // discretize
  const int horizontalSamples = imageWidth/regionWidth;
  const int verticalSamples = imageHeight/regionHeight;
  
    
  for (int rowIndex = 0; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples; ++colIndex)
    {
      unsigned char values = 0;
      
      for (int subRowIndex = 0; subRowIndex < regionHeight; ++subRowIndex)
      {
        for (int subColIndex = 0; subColIndex < regionWidth; ++subColIndex)
        {
          const int binIndex = static_cast<int>(CV_IMAGE_ELEM(discretizedAngles, float, rowIndex*regionHeight+subRowIndex, colIndex*regionWidth+subColIndex)*8.0f);
          
          if (binIndex <= 7)
          {
            values |= 1 << binIndex;
            //std::cerr << "_";
          }
        }
      }
      
      discretizedData[rowIndex*horizontalSamples+colIndex] = values;

      //if (values == 0)
      //{
      //  mask[rowIndex*horizontalSamples+colIndex] = 0;
      //}
    }
  }
  
  cvReleaseImage (&discretizedAngles);
}



