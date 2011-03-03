/*
 * masking.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */

/*
 * features.cpp
 *
 *  Created on: Nov 5, 2010
 *      Author: erublee
 */
#include <tod/training/masking.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using std::vector;
namespace tod
{

void GreenScreenMasker::mask(Features2d& features) const
{
  /** @TODO implement the green screen based mask */

}

FileMasker::FileMasker(const std::string& mask_file) :
  mask_(cv::imread(mask_file, CV_LOAD_IMAGE_GRAYSCALE))
{
}
FileMasker::FileMasker(const cv::Mat mask_image) :
  mask_(mask_image)
{

}
namespace
{
struct ContourData
{
  ContourData(const std::string& window_name, const cv::Mat& image) :
    window_name(window_name), image(image), finished(false)
  {
  }

  void clearImage()
  {
    image.copyTo(draw_image);
  }
  void drawContour()
  {
    for (int i = 0; i < int(points.size()) - 1; i++)
    {
      line(draw_image, points[i], points[i + 1], Scalar(255, 0, 0), 2, 8, 0);
    }
  }
  bool addPoint(const Point& pt)
  {

    if (!points.empty() && norm(pt - points[0]) < 10)
    {
      finished = true;
    }
    points.push_back(pt);
    return finished;
  }

  void createMask(cv::Mat& mask)
  {
    mask = Mat::zeros(image.size(), CV_8UC1);
    fillConvexPoly(mask, &(*points.begin()), points.size(), Scalar(255));
  }

  const std::string window_name;
  const cv::Mat image;
  cv::Mat draw_image;
  vector<Point> points;
  bool finished;
};

void contourOnMouse(int event, int x, int y, int flags, void* param)
{
  ContourData * cdata = reinterpret_cast<ContourData*> (param);

  if (x >= cdata->image.cols || y >= cdata->image.rows)
    return;

  if (event == CV_EVENT_LBUTTONDOWN)
  {

    if (cdata->finished)
    {
      cdata->points.clear();
      cdata->finished = false;
    }

    cdata->clearImage();

    cdata->addPoint(Point(x, y));
    cdata->drawContour();
    circle(cdata->draw_image, Point(x, y), 2, Scalar(255, 0, 0), 2, 8, 0);
    imshow(cdata->window_name, cdata->draw_image);
  }
}
}

void ContourMasker::mask(Features2d& features) const
{

  ContourData cdata("Countour Crop", features.image);
  namedWindow(cdata.window_name, CV_WINDOW_KEEPRATIO);
  imshow(cdata.window_name, features.image);
  setMouseCallback(cdata.window_name, contourOnMouse, &cdata);

  char c = 0;
  while (c != 's' || !cdata.finished)
  {
    c = char(waitKey(10));
    switch (c)
    {
      case 'r':
        cdata.points.clear();
        cdata.finished = false;
        break;
      default:
        break;
    }
  }
  cdata.createMask(features.mask);
}



}

