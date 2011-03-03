/*
 * select3dobj_masker.cpp
 *
 *  Created on: Nov 10, 2010
 *      Author: erublee
 */
#include "tod/training/masking.h"
#include "tod/training/pose.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using std::vector;
namespace
{

struct MouseEvent
{
  MouseEvent()
  {
    event = -1;
    buttonState = 0;
  }
  Point pt;
  int event;
  int buttonState;
};

void onMouse(int event, int x, int y, int flags, void* userdata)
{
  if (userdata)
  {
    MouseEvent* data = (MouseEvent*)userdata;
    data->event = event;
    data->pt = Point(x, y);
    data->buttonState = flags;
  }
}

Point3f image2plane(Point2f imgpt, const Mat& R, const Mat& tvec, const Mat& cameraMatrix, double Z)
{
  Mat R1 = R.clone();
  R1.col(2) = R1.col(2) * Z + tvec;
  Mat_<double> v = (cameraMatrix * R1).inv() * (Mat_<double> (3, 1) << imgpt.x, imgpt.y, 1);
  double iw = fabs(v(2, 0)) > DBL_EPSILON ? 1. / v(2, 0) : 0;
  return Point3f((float)(v(0, 0) * iw), (float)(v(1, 0) * iw), (float)Z);
}
Rect extract3DBox(const Mat& frame, const Mat& shown_frame_, Mat& mask, const Mat& cameraMatrix, const Mat& rvec,
                  const Mat& tvec, const vector<Point3f>& box, int nobjpt, bool runExtraSegmentation)
{
  mask = Mat::zeros(frame.size(), CV_8UC1);
  if (nobjpt == 0)
    return Rect();
  vector<Point3f> objpt;
  vector<Point2f> imgpt;

  objpt.push_back(box[0]);
  if (nobjpt > 1)
    objpt.push_back(box[1]);
  if (nobjpt > 2)
  {
    objpt.push_back(box[2]);
    objpt.push_back(objpt[2] - objpt[1] + objpt[0]);
  }
  if (nobjpt > 3)
    for (int i = 0; i < 4; i++)
      objpt.push_back(Point3f(objpt[i].x, objpt[i].y, box[3].z));

  projectPoints(Mat(objpt), rvec, tvec, cameraMatrix, Mat(), imgpt);
  if (shown_frame_.data)
  {
    Mat shownFrame = shown_frame_;
    if (nobjpt == 1)
      circle(shownFrame, imgpt[0], 3, Scalar(0, 255, 0), -1, CV_AA);
    else if (nobjpt == 2)
    {
      circle(shownFrame, imgpt[0], 3, Scalar(0, 255, 0), -1, CV_AA);
      circle(shownFrame, imgpt[1], 3, Scalar(0, 255, 0), -1, CV_AA);
      line(shownFrame, imgpt[0], imgpt[1], Scalar(0, 255, 0), 3, CV_AA);
    }
    else if (nobjpt == 3)
      for (int i = 0; i < 4; i++)
      {
        circle(shownFrame, imgpt[i], 3, Scalar(0, 255, 0), -1, CV_AA);
        line(shownFrame, imgpt[i], imgpt[(i + 1) % 4], Scalar(0, 255, 0), 3, CV_AA);
      }
    else
      for (int i = 0; i < 8; i++)
      {
        circle(shownFrame, imgpt[i], 3, Scalar(0, 255, 0), -1, CV_AA);
        line(shownFrame, imgpt[i], imgpt[(i + 1) % 4 + (i / 4) * 4], Scalar(0, 255, 0), 3, CV_AA);
        line(shownFrame, imgpt[i], imgpt[i % 4], Scalar(0, 255, 0), 3, CV_AA);
      }
  }

  if (nobjpt <= 2)
    return Rect();
  vector<Point> hull;
  convexHull(Mat_<Point> (Mat(imgpt)), hull);
  fillConvexPoly(mask, &hull[0], hull.size(), Scalar::all(255), 8, 0);
  Rect roi = boundingRect(Mat(hull)) & Rect(Point(), frame.size());
  return roi;
}

int select3DBox(const string& windowname, const string& selWinName, const Mat& frame, const Mat& cameraMatrix,
                const Mat& rvec, const Mat& tvec, vector<Point3f>& box)
{
  const float eps = 1e-3f;
  MouseEvent mouse;

  setMouseCallback(windowname, onMouse, &mouse);
  vector<Point3f> tempobj(8);
  vector<Point2f> imgpt(4), tempimg(8);
  vector<Point> temphull;
  int nobjpt = 0;
  Mat R, selectedObjMask, shownFrame;
  Rodrigues(rvec, R);
  box.resize(4);

  for (;;)
  {
    float Z = 0.f;
    bool dragging = (mouse.buttonState & CV_EVENT_FLAG_LBUTTON) != 0;
    int npt = nobjpt;

    if ((mouse.event == CV_EVENT_LBUTTONDOWN || mouse.event == CV_EVENT_LBUTTONUP || dragging) && nobjpt < 4)
    {
      Point2f m = mouse.pt;

      if (nobjpt < 2)
        imgpt[npt] = m;
      else
      {
        tempobj.resize(1);
        int nearestIdx = npt - 1;
        if (nobjpt == 3)
        {
          nearestIdx = 0;
          for (int i = 1; i < npt; i++)
            if (norm(m - imgpt[i]) < norm(m - imgpt[nearestIdx]))
              nearestIdx = i;
        }

        if (npt == 2)
        {
          float dx = box[1].x - box[0].x, dy = box[1].y - box[0].y;
          float len = 1.f / std::sqrt(dx * dx + dy * dy);
          tempobj[0] = Point3f(dy * len + box[nearestIdx].x, -dx * len + box[nearestIdx].y, 0.f);
        }
        else
          tempobj[0] = Point3f(box[nearestIdx].x, box[nearestIdx].y, 1.f);

        projectPoints(Mat(tempobj), rvec, tvec, cameraMatrix, Mat(), tempimg);

        Point2f a = imgpt[nearestIdx], b = tempimg[0], d1 = b - a, d2 = m - a;
        float n1 = (float)norm(d1), n2 = (float)norm(d2);
        if (n1 * n2 < eps)
          imgpt[npt] = a;
        else
        {
          Z = d1.dot(d2) / (n1 * n1);
          imgpt[npt] = d1 * Z + a;
        }
      }
      box[npt] = image2plane(imgpt[npt], R, tvec, cameraMatrix, npt < 3 ? 0 : Z*10);

      if ((npt == 0 && mouse.event == CV_EVENT_LBUTTONDOWN) || (npt > 0 && norm(box[npt] - box[npt - 1]) > eps
          && mouse.event == CV_EVENT_LBUTTONUP))
      {
        nobjpt++;
        if (nobjpt < 4)
        {
          imgpt[nobjpt] = imgpt[nobjpt - 1];
          box[nobjpt] = box[nobjpt - 1];
        }
      }

      // reset the event
      mouse.event = -1;
      //mouse.buttonState = 0;
      npt++;
    }

    frame.copyTo(shownFrame);
    extract3DBox(frame, shownFrame, selectedObjMask, cameraMatrix, rvec, tvec, box, npt, false);

    imshow(windowname, shownFrame);
    imshow(selWinName, selectedObjMask);

    char c = waitKey(30);
    if ((c & 255) == 27)
    {
      nobjpt = 0;
    }
    if (c == 'q' || c == 'Q' || c == ' ')
    {
      box.clear();
      return c == ' ' ? -1 : -100;
    }
    if ((c == '\r' || c == '\n') && nobjpt == 4 && box[3].z != 0)
      return 1;
  }
}

bool readModelViews(const string& filename, vector<Point3f>& box, vector<string>& imagelist, vector<Rect>& roiList,
                    vector<Vec6f>& poseList)
{
  imagelist.resize(0);
  roiList.resize(0);
  poseList.resize(0);
  box.resize(0);

  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["box"] >> box;

  FileNode all = fs["views"];
  if (all.type() != FileNode::SEQ)
    return false;
  FileNodeIterator it = all.begin(), it_end = all.end();

  for (; it != it_end; ++it)
  {
    FileNode n = *it;
    imagelist.push_back((string)n["image"]);
    FileNode nr = n["rect"];
    roiList.push_back(Rect((int)nr[0], (int)nr[1], (int)nr[2], (int)nr[3]));
    FileNode np = n["pose"];
    poseList.push_back(Vec6f((float)np[0], (float)np[1], (float)np[2], (float)np[3], (float)np[4], (float)np[5]));
  }

  return true;
}

bool writeModelViews(const string& filename, const vector<Point3f>& box, const vector<string>& imagelist, const vector<
    Rect>& roiList, const vector<Vec6f>& poseList)
{
  FileStorage fs(filename, FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  fs << "box" << "[:";
  fs << box << "]" << "views" << "[";

  size_t i, nviews = imagelist.size();

  CV_Assert( nviews == roiList.size() && nviews == poseList.size() )
    ;

  for (i = 0; i < nviews; i++)
  {
    Rect r = roiList[i];
    Vec6f p = poseList[i];

    fs << "{" << "image" << imagelist[i] << "roi" << "[:" << r.x << r.y << r.width << r.height << "]" << "pose" << "[:"
        << p[0] << p[1] << p[2] << p[3] << p[4] << p[5] << "]" << "}";
  }
  fs << "]";

  return true;
}

}

namespace tod
{
Select3dObjecMasker::Select3dObjecMasker(const Camera& camera) :
  camera_(camera)
{

}
bool Select3dObjecMasker::guiSelectBox(const Features2d& features)
{
  PoseRT pose = features.camera.pose;
  namedWindow("View", CV_WINDOW_KEEPRATIO);
  namedWindow("Selected Object", CV_WINDOW_KEEPRATIO);

  cout << "Please select a 3d box.\n"
      << "\tleft click - vertex\n"
      << "\tleft drag - drag vertex (use this for every vertex besides first)\n"
      << "\tenter - save box\n"
      << "\tesc - reset box\n"
      << "\tq - quit\n"
      << "\tspace - skip image\n"
      << endl;
  cv::Mat image = features.image;
  float scale_factor = CheckerboardPoseEstimator::ScaleImage(image, 400);
 // scale_factor *= image.size().width /float(camera_.image_size.width);

  Mat_<double> K = CheckerboardPoseEstimator::ScaleK(camera_.K, scale_factor);

  int code = select3DBox("View", "Selected Object", image, K, pose.rvec, pose.tvec, box_);

  if (code != -1)
  {
    destroyWindow("View");
    destroyWindow("Selected Object");
    waitKey(10);
  }

  if (code == -100)
    return false;
  else
    return true;
}
void Select3dObjecMasker::setBox(const std::vector<cv::Point3f>& box)
{
  box_ = box;
}
const std::vector<cv::Point3f>& Select3dObjecMasker::getBox() const
{
  return box_;
}

void Select3dObjecMasker::mask(Features2d& features) const
{
  if (box_.empty())
    return;
  PoseRT pose = features.camera.pose;
  //get some lower res images to work with
  cv::Mat low_res_color = features.image;
  float scale_factor = CheckerboardPoseEstimator::ScaleImage(low_res_color, 400);

  Mat low_res_mask;
  Mat_<double> K = CheckerboardPoseEstimator::ScaleK(camera_.K, scale_factor);
  Rect roi = extract3DBox(low_res_color, Mat(), low_res_mask, K, pose.rvec, pose.tvec, box_, 4, false);

  bitwise_and(low_res_mask, GC_PR_FGD, low_res_mask);
//  Mat roi_mat = low_res_mask(roi);
//  bitwise_or(roi_mat,GC_PR_BGD,roi_mat);

  Mat bgdModel, fgdModel;
  grabCut(low_res_color, low_res_mask, roi, bgdModel, fgdModel,2, GC_INIT_WITH_MASK);
  low_res_mask = (low_res_mask & GC_FGD) * 255;
  resize(low_res_mask, features.mask, features.image.size());

//  imshow("image", low_res_color);
//  imshow("mask", low_res_mask);
//  waitKey(10);

}

}
