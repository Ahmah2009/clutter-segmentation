#include "textured_object_detection/training_set3d.h"
#include <iostream>
#include <visualization_msgs/Marker.h>
#include "posest/pnp_ransac.h"
#include "stereo_object_recognition/RANSACRigidTransformationEstimator.h"
#include "stereo_object_recognition/SVDRigidEstimator.h"
using namespace std;
using namespace cv;
using namespace stereo_object_recognition;
#define RANSAC_ITERATIONS 1000
#define THRESHOLD3D 0.000005

enum flann_distance_t
{
  EUCLIDEAN = 1, MANHATTAN = 2, MINKOWSKI = 3
};

extern "C"
{
void flann_set_distance_type(flann_distance_t distance_type, int order);
}

TrainingSet3d::TrainingSet3d(std::string dir, std::string configPath) : TrainingSet(dir, configPath)
{
}

TrainingSet3d::~TrainingSet3d()
{
}

void TrainingSet3d::detectAndMatchPoints(const cv::Mat& img, const cv::Mat& mask,
                                         std::vector<cv::KeyPoint>& testKeypoints, std::vector<DMatch>& matches)
{
  //detect keypoints for test image
    detector->detect(img, testKeypoints);
    vector<KeyPoint>::iterator it;
      for (it = testKeypoints.begin(); it != testKeypoints.end();)
      {
        if (mask.at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) == 0)
        {
          it = testKeypoints.erase(it);
        }
        else
        {
          ++it;
        }
      }
    Mat testDescriptors;
    //compute descriptors for test image
    descriptorExtractor->compute(img, testKeypoints, testDescriptors);
    cout << "Extracted " << testKeypoints.size() << " points" << endl;

    //match test image descriptors to train base descriptors
    flann_set_distance_type(MANHATTAN, 0);
    Mat m_indices(testDescriptors.rows, params.knnNum, CV_32S);
    Mat m_dists(testDescriptors.rows, params.knnNum, CV_32F);
    flann_index->knnSearch(testDescriptors, m_indices, m_dists, params.knnNum, cv::flann::SearchParams(64)); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int> (0);
    float* dists_ptr = m_dists.ptr<float> (0);
    int obj_ind, img_ind, ind, j;

    for (int i = 0; i < m_indices.rows; ++i)
    {
      obj_ind = obj_indices[indices_ptr[i * params.knnNum]];
      img_ind = img_indices[indices_ptr[i * params.knnNum]];

      for (j = 1; j < params.knnNum; ++j)
      {
        if (obj_indices[indices_ptr[i * params.knnNum + j]] == obj_ind)
        {
          break;
        }
      }

      if (j < params.knnNum)
        ind = j;
      else
        ind = 1;

      if (dists_ptr[params.knnNum * i] < params.ratioTestThreshold * dists_ptr[params.knnNum * i + ind])
      {
        DMatch match;
        match.queryIdx = i;
        match.trainIdx = indices_ptr[i * params.knnNum];
        matches.push_back(match);
      }
    }

    if (isPrintResults)
      cout << "Matched " << matches.size() << " points" << endl;
}

void TrainingSet3d::findPoseGuesses(Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<
    cv::Point3f>& test3dPoints, const std::vector<cv::DMatch>& obj_matches,
                                    const std::vector<std::vector<int> >& indices, const cv::Mat& img, std::vector<
                                        PoseGuess>& poseGuesses)
{
  // loop over clusters, doing RANSAC on each
    for (size_t k = 0; k < indices.size(); k++)
    {
      vector<Point2f> imagePoints;
      vector<Point2f> objPoints;
      vector<Point3f> testObjectPoints;
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
        testObjectPoints.push_back(test3dPoints[obj_matches[l].queryIdx]);
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

      vector<int> activePoints;
      activePoints.assign(indices[k].size(), 1);

      while(*std::max_element(activePoints.begin(), activePoints.end()) > 0)
      {
        imagePoints.clear();
        objPoints.clear();
        objectPoints.clear();
        testObjectPoints.clear();
        imageIndexes.clear();
        globalIndices.clear();

        int l;
        Mat drawImg;
        object->images[0].copyTo(drawImg);

        vector<int> acivePointsIndices;

        for (size_t m = 0; m < indices[k].size(); m++)
        {
          if(activePoints[m] == 0) continue;

          acivePointsIndices.push_back(m);
          int img_ind = img_indices[obj_matches[indices[k][m]].trainIdx];
          l = indices[k][m];
          int ind = object->pointsIndices[img_ind][keypoint_indices[obj_matches[l].trainIdx]];

          imagePoints.push_back(Point2f(keypoints[obj_matches[l].queryIdx].pt.x,
                  keypoints[obj_matches[l].queryIdx].pt.y));
          testObjectPoints.push_back(test3dPoints[obj_matches[l].queryIdx]);
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

          if(img_ind == 0)
          {
            vector<Point2f> projectedPoint;
            projectPoints(Mat(point), object->rvecs[img_ind], object->tvecs[img_ind], cameraMatrix, distCoeffs, projectedPoint);
            circle(drawImg, Point(projectedPoint[0].x, projectedPoint[0].y), 5, cvScalar(0));
          }
        }

        printf("Running cluster %d with %d points\n", (int)k, (int)objectPoints.size());

        if(isDrawInliers && !isNode)
        {
          namedWindow("projection img1", 1);
          imshow("projection img1", drawImg);
        }
        //      waitKey(0);
  #endif

      vector<int> inliers;
      Mat rvec, tvec;
      //run solvePnPRansac for finding inliers (using point clouds from train base)
      //solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, RANSAC_ITERATIONS,
      //             params.reprojectErrorThreshold, 100, &inliers);


      SVDRigidEstimator estimator;
      RANSACRigidTransformationEstimator ransacEstimator( estimator, THRESHOLD3D, 0.999, 0.5, 1.5);
      std::vector<std::pair<cv::Point3d,cv::Point3d> > pairs;
      for (size_t pairIndex = 0; pairIndex < testObjectPoints.size(); pairIndex++)
      {
        pairs.push_back(make_pair(Point3d(objectPoints[pairIndex].x, objectPoints[pairIndex].y, objectPoints[pairIndex].z),
             Point3d(testObjectPoints[pairIndex].x, testObjectPoints[pairIndex].y, testObjectPoints[pairIndex].z)));
      }
      Mat R;
      Vec3d t;
      bool isEstimateSuccess = ransacEstimator.estimate( pairs, R, t);
      cout << "RANSAC estimation status = " << isEstimateSuccess << endl;
      if (isEstimateSuccess)
      {
        rvec.create(3, 1, CV_64FC1);
        tvec.create(3, 1, CV_64FC1);
        tvec.at<double> (0, 0) = t[0];
        tvec.at<double> (1, 0) = t[1];
        tvec.at<double> (2, 0) = t[2];
        Rodrigues(R, rvec);
        vector<unsigned> estInliers = ransacEstimator.getInliers();
        for (size_t inliersIndex = 0; inliersIndex < estInliers.size(); inliersIndex++)
        {
          inliers.push_back(estInliers[inliersIndex]);
        }
      }


  #if 1
      if (inliers.size() > (size_t)params.minInliersCount && isEstimateSuccess)
      {
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

          if (img_ind == 1)
          {
            circle(drawImg, Point(projectedPoints[inliers[i]].x, projectedPoints[inliers[i]].y), 5, Scalar(255));
            //cross(drawImg, Point(imagePoints[inliers[i]].x, imagePoints[inliers[i]].y), Scalar(0), 5);
          }
        }
        printf("reprojection error: %f\n", sqrt(sum / inliers.size()));

        Mat drawImgS;
        resize(drawImg, drawImgS, Size(drawImg.cols * params.scale, drawImg.rows * params.scale));
        if (isDrawInliers && !isNode)
        {
          namedWindow("inliers 0", 1);
          imshow("inliers 0", drawImgS);
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

        //        waitKey(0);
      }
  #endif
      if (isPrintResults)
        cout << "Inliers count = " << inliers.size() << endl;

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
      const int roughMinInliersCount = params.minInliersCount/2;
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

void TrainingSet3d::refinePoses(const Object* object, const std::vector<cv::KeyPoint>& keypoints, const std::vector<
    cv::Point3f>& test3dPoints, const std::vector<cv::DMatch>& matches, const cv::Mat& img,
                                std::vector<PoseGuess>& guesses)
{
    vector<Point2f> imagePoints;
    vector<Point3f> objectPoints;
    vector<Point3f> testObjectPoints;
    vector<Point2f> objPoints;
    imagePoints.resize(matches.size());
    objectPoints.resize(matches.size());
    testObjectPoints.resize(matches.size());
    objPoints.resize(matches.size());
    for(size_t i = 0; i < matches.size(); i++)
    {
      int img_ind = img_indices[matches[i].trainIdx];
      int point_ind = object->pointsIndices[img_ind][keypoint_indices[matches[i].trainIdx]];

      imagePoints[i] = Point2f(keypoints[matches[i].queryIdx].pt.x,
              keypoints[matches[i].queryIdx].pt.y);
      testObjectPoints[i] = test3dPoints[matches[i].queryIdx];
      objPoints[i] = Point2f(object->keypoints[img_ind][keypoint_indices[matches[i].trainIdx]].pt.x,
                                  object->keypoints[img_ind][keypoint_indices[matches[i].trainIdx]].pt.y);
      vector<Point3f> point;
      point.push_back(Point3f(object->clouds[img_ind].points[point_ind].x, object->clouds[img_ind].points[point_ind].y,
              object->clouds[img_ind].points[point_ind].z));
      vector<Point3f> rotated_point;
      project3dPoints(point, object->rvecs[img_ind], object->tvecs[img_ind], rotated_point);
      objectPoints[i] = rotated_point[0];
    }

    for(size_t i = 0; i < guesses.size(); i++)
    {
      // find inliers in global match array
      //vector<Point2f> projectedPoints;
      //projectPoints(Mat(objectPoints), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);
      vector<Point3f> rotatedPoints;
      project3dPoints(objectPoints, guesses[i].rvec, guesses[i].tvec, rotatedPoints);

      vector<Point2f> inlierImagePoints;
      vector<Point3f> inlierObjectPoints;
      vector<Point3f> inlierTestObjectPoints;
      vector<int> inlierIndices;
      cout << "guesses[i].inliers size = " <<  guesses[i].inliers.size() << endl;
      for(size_t j = 0; j < rotatedPoints.size(); j++)
      {
        float dist = norm(rotatedPoints[j] - testObjectPoints[j])*norm(rotatedPoints[j] - testObjectPoints[j]);
        if(dist < THRESHOLD3D*5)
        {
          inlierImagePoints.push_back(imagePoints[j]);
          inlierObjectPoints.push_back(objectPoints[j]);
          inlierTestObjectPoints.push_back(testObjectPoints[j]);
          inlierIndices.push_back(j);
        }
      }

      vector<Point2f> hull;
      convexHull(Mat(inlierImagePoints), hull);

      float stddev = contourArea(Mat(hull));

      vector<int> inliers;
      const float fineReprojectErrorThreshold = 8.0f;
      //solvePnPRansac(inlierObjectPoints, inlierImagePoints, cameraMatrix, distCoeffs, guesses[i].rvec, guesses[i].tvec, true,
        //           RANSAC_ITERATIONS*10, fineReprojectErrorThreshold, params.minInliersCount, &inliers);

       cout << "inlierIndices.size = " << inlierIndices.size() << endl;
       SVDRigidEstimator estimator;
       RANSACRigidTransformationEstimator ransacEstimator( estimator, THRESHOLD3D/30, 0.999, 0.01, 1.05);
       std::vector<std::pair<cv::Point3d,cv::Point3d> > pairs;
       for (size_t pairIndex = 0; pairIndex < inlierObjectPoints.size(); pairIndex++)
       {
         pairs.push_back(make_pair(Point3d(inlierObjectPoints[pairIndex].x, inlierObjectPoints[pairIndex].y, inlierObjectPoints[pairIndex].z),
              Point3d(inlierTestObjectPoints[pairIndex].x, inlierTestObjectPoints[pairIndex].y, inlierTestObjectPoints[pairIndex].z)));
       }
       Mat R;
       Vec3d t;
       bool isEstimateSuccess = ransacEstimator.estimate( pairs, R, t);
       cout << "REFINE RANSAC estimation status = " << isEstimateSuccess << endl;
       if (isEstimateSuccess)
       {
         guesses[i].rvec.create(3, 1, CV_64FC1);
         guesses[i].tvec.create(3, 1, CV_64FC1);
         guesses[i].tvec.at<double> (0, 0) = t[0];
         guesses[i].tvec.at<double> (1, 0) = t[1];
         guesses[i].tvec.at<double> (2, 0) = t[2];
         Rodrigues(R, guesses[i].rvec);
         vector<unsigned> estInliers = ransacEstimator.getInliers();
         for (size_t inliersIndex = 0; inliersIndex < estInliers.size(); inliersIndex++)
         {
           inliers.push_back(estInliers[inliersIndex]);
         }
       }
       else
       {
         guesses[i].stddev = -1.0;
         continue;
       }


      guesses[i].fineInliersCount = inliers.size();
      // copy inlier indices into the global point array
      guesses[i].inliers.resize(inliers.size());
      for(size_t j = 0; j < inliers.size(); j++)
      {
        guesses[i].inliers[j] = inlierIndices[inliers[j]];
      }
      guesses[i].imagePoints = imagePoints;
      guesses[i].objPoints = objPoints;
      guesses[i].img_idx = 0;
      guesses[i].stddev = stddev;

      if(isPrintResults)
      {
        cout << "Guess " << i << ":" << guesses[i].inliers.size() << " rough inliers, " << inliers.size() << " refined inliers" << ", cluster stddev " << stddev << endl;
      }

      if(isDrawInliers)
      {
        Mat drawImg = img.clone();
        for(size_t i = 0; i < inliers.size(); i++)
        {
          circle(drawImg, Point(inlierImagePoints[inliers[i]].x, inlierImagePoints[inliers[i]].y), 5, cvScalar(255));
        }

        Mat drawImgS;
        resize(drawImg, drawImgS, Size(drawImg.cols*params.scale, drawImg.rows*params.scale));

        namedWindow("fine inliers", 1);
        imshow("fine inliers", drawImgS);
        waitKey(0);
      }
  }
}

void TrainingSet3d::recognize(const cv::Mat& img, const cv::Mat& mask, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                              std::vector<ObjectInfo>& res_objects, const ros::Publisher &points_pub)
{
    assert(maxImgNum != 0);
    //clear results vector
    res_objects.clear();
    cvtColor(img, bbProjImg, CV_GRAY2RGB);

    vector<KeyPoint> keypoints;
    vector<DMatch> matches;
    //detect keypoints, compute descriptors and match them
    detectAndMatchPoints(img, mask, keypoints, matches);
    if (keypoints.empty())
      return;

    vector<Point3f> testObjectPoints;
    for (vector<KeyPoint>::const_iterator it = keypoints.begin(); it != keypoints.end(); it++)
    {
      uint16_t index = mask.at<uint16_t> ((int)(*it).pt.y, (int)(*it).pt.x) - 1;
      testObjectPoints.push_back(Point3f(cloud.points[index].x, cloud.points[index].y, cloud.points[index].z));
    }

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
      //vector<Point2f> imagePoints;
  //    vector<vector<Point2f> > bestImagePoints;
      //vector<Point2f> objPoints;
  //    vector<vector<Point2f> > bestObjPoints;
      //vector<Point3f> objectPoints;

  //    vector<vector<Point3f> > bestObjectPoints;
  //    vector<vector<int> > bestInliers;
  //    vector<int> lind;
  //    vector<vector<int> > bestImageIndexes;
      vector<int> imageIndexes;
      vector<Mat> rvecl, tvecl;

      Object* object = objects[objectIndex];

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

      //cluster these points (distance between points matched to different images is infinity)
      clusterPoints(imgPoints, imgIndexes, params.clusterThreshold, indices);

      if (isDrawClusters)
        drawClusters(img, imgPoints, indices);

      vector<PoseGuess> guesses;
      findPoseGuesses(object, keypoints, testObjectPoints, obj_matches, indices, img, guesses);
      refinePoses(object, keypoints, testObjectPoints, obj_matches, img, guesses);


      // filter bad guesses
      vector<PoseGuess> filtered;
      for(size_t i = 0; i < guesses.size(); i++)
      {
        if(guesses[i].fineInliersCount < params.minInliersCount)
          {
             cout << i << " filtered inliers count" << endl;
             continue;
          }
        const float minStddev = 1000.0f;
        if(guesses[i].stddev < minStddev || guesses[i].stddev < 0)
          {

            cout << i << " filtered stddev" << endl;
            continue;
          }

        filtered.push_back(guesses[i]);
      }
      guesses = filtered;


      // filter overlappping guesses
      filterOverlappingPoses(imgPoints, guesses);


      cout << "inliers size " << guesses.size() << endl;
      //if inliers number for this object >  MIN_INLIERS_COUNT we think, that we find object
      for (size_t i = 0; i < guesses.size(); i++)
      {
        vector<Point3f> points;
        for (size_t m = 0; m < object->clouds[guesses[i].img_idx].points.size(); m++)
        {
          points.push_back(Point3f(object->clouds[guesses[i].img_idx].points[m].x, object->clouds[guesses[i].img_idx].points[m].y,
                                   object->clouds[guesses[i].img_idx].points[m].z));
        }
        if (isPrintResults)
        {
          cout << "rvec: " << guesses[i].rvec.at<double> (0, 0) << " " << guesses[i].rvec.at<double> (1, 0) << " " << guesses[i].rvec.at<
              double> (2, 0) << endl;
          cout << "tvec: " << guesses[i].tvec.at<double> (0, 0) << " " << guesses[i].tvec.at<double> (1, 0) << " " << guesses[i].tvec.at<
              double> (2, 0) << endl;
        }
        vector<Point3f> rotated_object_points;
        //apply found R and T to point cloud
        project3dPoints(points, guesses[i].rvec, guesses[i].tvec, rotated_object_points);

        vector<Point2f> projectedPoints;
        projectPoints(Mat(points), guesses[i].rvec, guesses[i].tvec, cameraMatrix, distCoeffs, projectedPoints);
        double zavg = 0.0;
        for (size_t m = 0; m < rotated_object_points.size(); m++)
        {
          zavg += rotated_object_points[m].z;
          //        printf("point %f %f %f projected into %f %f\n", points[m].x, points[m].y, points[m].z, projectedPoints[m].x, projectedPoints[m].y);
        }

        if (zavg < 0)
        {
          ROS_INFO("Registered pose behind the camera, skipping");
          continue;
          //        assert(0);
          for (size_t m = 0; m < rotated_object_points.size(); m++)
          {
            rotated_object_points[m] *= -1.0;
          }
          //continue; //TODO: fix it
        }
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
        res_objects.push_back(found_object);
      }
      /*    else
       {
       vector<Point3f> rotated_object_points;
       publishPoints(rotated_object_points, points_pub, j);
       }
       */
    }

    publishPoints(fullObjectPoints, points_pub, fullObjectIndices);

    if (isDrawProjection && !isNode)
    {
      namedWindow("projection");
      Mat smallProjImg;
      resize(projImg, smallProjImg, Size(), params.scale, params.scale);
      imshow("projection", smallProjImg);
      waitKey(0);
      waitKey(0);
    }
    if (isDrawCorrespondence)
      drawCorrespondence(img, keypoints, matches);
}
