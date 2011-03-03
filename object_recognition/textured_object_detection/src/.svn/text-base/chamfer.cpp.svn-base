#include "textured_object_detection/chamfer.hpp"
using namespace cv;

GMatchCM::GMatchCM(GMatchCM::Params _edges_params) :
  edges_params(_edges_params)
{
}

void GMatchCM::setTemplateEdgels(const vector<Point3f>& _edgels)
{
  edgels = _edgels;
}

void GMatchCM::setTestImage(const Mat& src)
{
  Mat edges;
  Canny(src, edges, edges_params.canny_thresh1, edges_params.canny_thresh2);
  cropImage(edges, 1);
  test_image = edges;
}

float GMatchCM::calcMatchCost(const GMap3D& gmap) const
{
  assert( !test_image.empty() );
  assert( !edgels.empty() );

  Mat chamferTemplate;
  Point offset;
  mapEdgels(gmap, chamferTemplate, offset);

  ChamferMatching chamferMatching(false);
  cropImage(chamferTemplate, 1);
  IplImage tpl = chamferTemplate;
  Mat chamferTemplateClone = chamferTemplate.clone();
  IplImage tpl2 = chamferTemplateClone;
  chamferMatching.addTemplateFromImage(&tpl);
  ChamferTemplate tmpTemplate(&tpl2, 1);

  IplImage testImageIpl = test_image;
  vector < CvPoint > locations;
  locations.push_back(cvPoint(offset.x + tmpTemplate.center.x, offset.y + tmpTemplate.center.y));
  LocationImageRange locationImageRange(locations, 1, 1, 1);
  ChamferMatch chamferMatch = chamferMatching.matchEdgeImage(&testImageIpl, locationImageRange, 0, 1);

  ChamferMatch::ChamferMatches chamferMatches = chamferMatch.getMatches();
  if (chamferMatches[0].tpl == 0)
    return -1;
  return chamferMatches[0].cost;
}

void GMatchCM::showMatch(const GMap3D& gmap) const
{
  Mat tpl;
  Point offset;
  mapEdgels(gmap, tpl, offset);

  Mat vis;
  cvtColor(test_image, vis, CV_GRAY2BGR);

  assert( tpl.type() == CV_8UC1 );
  for (int i = 0; i < tpl.rows; i++)
  {
    for (int j = 0; j < tpl.cols; j++)
    {
      if (tpl.at<uchar> (i, j) != 0)
      {
        circle(vis, offset + Point(j, i), 1, Scalar(0, 0, 255));
      }
    }
  }

  imshow("Edgels projection", vis);
}

void GMatchCM::cropImage(Mat &src, int width)
{
  assert(src.type() == CV_8UC1);

  for (int col = 0; col < src.cols; col++)
  {
    for (int k = 0; k < width; k++)
    {
      src.at<uchar> (k, col) = 0;
      src.at<uchar> (src.rows - 1 - k, col) = 0;
    }
  }

  for (int row = 0; row < src.rows; row++)
  {
    for (int k = 0; k < width; k++)
    {
      src.at<uchar> (row, k) = 0;
      src.at<uchar> (row, src.cols - 1 - k) = 0;
    }
  }
}

void GMatchCM::mapEdgels(const GMap3D &gmap, Mat &dstImage, Point &offset) const
{
  vector < Point2f > allMappedEdgels;
  vector < Point > mappedEdgels;
  gmap.mapPoints(edgels, allMappedEdgels);
  for (size_t j = 0; j < allMappedEdgels.size(); j++)
  {
    Point edgel = allMappedEdgels[j];
    if (edgel.x < 0 || edgel.x >= test_image.cols || edgel.y < 0 || edgel.y >= test_image.rows)
      continue;
    //TODO: filter outliers
    mappedEdgels.push_back(edgel);
  }

  Rect rect = boundingRect(Mat(mappedEdgels));
  dstImage = Mat(rect.size(), CV_8UC1, Scalar(0));
  for (size_t ptIdx = 0; ptIdx < mappedEdgels.size(); ptIdx++)
  {
    Point pt = Point(mappedEdgels[ptIdx].x - rect.x, mappedEdgels[ptIdx].y - rect.y);
    dstImage.at<uchar> (pt.y, pt.x) = 255;
  }
  offset = rect.tl();
}
