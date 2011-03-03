#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chamfer_matching/chamfer_matching.h>

using namespace cv;

Mat extractEdges(const Mat& src)
{
  Mat grey;
  if(src.channels() == 3)
  {
    cvtColor(src, grey, CV_RGB2GRAY);
  }
  else
  {
    grey = src;
  }

  Mat edges;
  Canny(grey, edges, 80, 120);
  return edges;
}

int main(int argc, char** argv)
{
  if(argc != 3)
  {
    printf("Usage: shape_match <image1> <image2>\n");
    return 0;
  }

  // load the training image
  Mat img1 = imread(argv[1]);
  Mat edges1 = extractEdges(img1);

  namedWindow("edges1", 1);
  imshow("edges1", edges1);
  waitKey(0);

  ChamferMatching cmatch;
  IplImage _edges1 = edges1;
  cmatch.addTemplateFromImage(&_edges1);

  Mat img2 = imread(argv[2]);
  Mat edges2 = extractEdges(img2);

  // run the chamfer matching
  SlidingWindowImageRange range(edges1.cols, edges1.rows);
  IplImage _edges2 = edges2;
  ChamferMatch match = cmatch.matchEdgeImage(&_edges2, range);

  Mat display;
  printf("Found %d matches\n", match.getMatches().size());
  for(size_t i = 0; i < match.getMatches().size(); i++)
  {
    img2.copyTo(display);
    IplImage _display = display;
    match.showMatch(&_display, int(i));

    printf("Match %d, cost %f\n", int(i), match.getMatches()[i].cost);

    namedWindow("match", 0);
    imshow("match", display);
    waitKey(0);
  }
}
