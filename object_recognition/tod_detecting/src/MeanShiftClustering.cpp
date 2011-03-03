/*
 * MeanShiftClustering.cpp
 *
 *  Created on: Jan 4, 2011
 *      Author: erublee
 */

#include <tod/detecting/clustering.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <functional>
using namespace std;

using namespace cv;
namespace tod
{

void MeanShiftParams::write(cv::FileStorage& fs) const
{
  cvWriteComment(*fs, "MeanShiftParams", 0);

  fs << "{";
  cvWriteComment(*fs, "number of iterations before stopping clustering algorithm", 0);
  fs << "iterations" << iterations;
  cvWriteComment(
                 *fs,
                 "this is the radius of the the clustering ball or window, either in pixels or in 3d units(depends on 3d data mm,cm,meters)",
                 0);
  fs << "lambda" << lambda;
  cvWriteComment(*fs, "this is the point at which to considered the clustering algorithm finished.", 0);
  fs << "epsilon" << epsilon;
  cvWriteComment(*fs, "If 0, will do 2d clustering, if 1 will use 3d clustering", 0);
  fs << "use3d" << (int)use3d;
  fs << "}";
}
void MeanShiftParams::read(const cv::FileNode& fn)
{
  iterations = (int)fn["iterations"];
  lambda = (double)fn["lambda"];
  epsilon = (double)fn["epsilon"];
  use3d = (int)fn["use3d"];
}
namespace
{

struct NormDot
{
  template<typename VectorT>
    typename VectorT::value_type operator()(const VectorT& v) const
    {
      return v.dot(v);
    }
};

struct FlatKernel
{
  double lambda_squared;
  typedef int ResultType;
  FlatKernel(float lambda) :
    lambda_squared(lambda * lambda)
  {

  }
  template<typename VectorT, class NormF>
    ResultType operator()(VectorT x, NormF f) const
    {
      return f(x) < lambda_squared ? 1 : 0;
    }
};
template<typename VectorTIt, typename Kernel, class NormF>
  struct SampleMean
  {
    VectorTIt begin;
    VectorTIt end;
    Kernel K;
    NormF f;
    typedef typename Kernel::ResultType KernelResultType;
    SampleMean(VectorTIt begin, VectorTIt end, Kernel K, const NormF& f = NormF()) :
      begin(begin), end(end), K(K), f(f)
    {

    }
    template<typename VectorT>
      VectorT operator()(const VectorT& x) const
      {
        VectorTIt s = begin;
        KernelResultType sumK = KernelResultType();
        VectorT sumS;
        while (s != end)
        {
          VectorT xs = x - *s;
          KernelResultType k = K(xs, f);
          sumK += k;
          sumS += k * (*s);
          ++s;
        }
        return (1. / sumK) * sumS;
      }
  };

template<typename VectorTIt, typename Kernel, class NormF>
  struct MeanShift
  {
    VectorTIt begin;
    VectorTIt end;
    VectorTIt begin_out;
    VectorTIt end_out;
    typedef typename VectorTIt::value_type VectorT;
    Kernel K;
    NormF f;
    MeanShift(VectorTIt begin, VectorTIt end, VectorTIt begin_out, VectorTIt end_out, Kernel K, NormF f = NormF()) :
      begin(begin), end(end), begin_out(begin_out), end_out(end_out), K(K), f(f)
    {
    }
    void iterate(const int N, double escape_eps)
    {
      double current_mag = numeric_limits<double>::max();
      for (int i = 0; i < N && current_mag > escape_eps * escape_eps; i++)
      {
        VectorTIt s = begin;
        VectorTIt s_ = begin_out;
        SampleMean<VectorTIt, Kernel, NormF> m(begin, end, K, f);
        double mag_mean = 0;
        int count = 0;
        while (s != end)
        {
          *s_ = m(*s);
          VectorT ms = *s - *s_;
          mag_mean += f(ms);
          count++;
          ++s;
          ++s_;
        }
        current_mag = mag_mean / (double)count;
        cout << "current_mag:" << sqrt(current_mag) << endl;

        std::swap(begin, begin_out);
        std::swap(end, end_out);
      }

      std::swap(begin, begin_out);
      std::swap(end, end_out);
    }
  };

template<typename VectorTIt, typename Kernel, class NormF>
  struct Cluster
  {
    VectorTIt begin;
    VectorTIt end;

    typedef typename VectorTIt::value_type VectorT;

    Kernel K;
    NormF f;
    Cluster(VectorTIt begin, VectorTIt end, Kernel K, NormF f = NormF()) :
      begin(begin), end(end), K(K), f(f)
    {
    }

    template<typename LabelsTIt>
      int operator()(LabelsTIt begin_label) const
      {
        VectorTIt s = begin;
        typedef typename LabelsTIt::value_type LabelT;
        LabelT label = LabelT();
        label++; //prevent 0

        while (s != end)
        {
          if (*begin_label == LabelT())
          {
            *begin_label = label++;
            VectorTIt ss = s;
            LabelsTIt lb = begin_label;
            while (ss != end)
            {
              if (*lb == LabelT())
              {
                VectorT ds = *s - *ss;
                if (K(ds, f))
                {
                  *lb = *begin_label;
                }
              }
              ++lb;
              ++ss;
            }
          }
          ++s;
          ++begin_label;
        }

        return label;
      }

  };

}

MeanShiftClustering::MeanShiftClustering(const MeanShiftParams& params) :
  params_(params)
{

}

void MeanShiftClustering::cluster(const Features3d& features, std::vector<Features3d>& clusters)
{
  std::vector<Point2f> points;
  std::vector<Point2f> working;
  std::vector<Point2f> meanshifted;

  cv::KeyPoint::convert(features.features().keypoints, points);
  working = points;
  meanshifted.resize(working.size());
  FlatKernel K(params_.lambda);
  MeanShift<vector<Point2f>::iterator, FlatKernel, NormDot> ms(working.begin(), working.end(), meanshifted.begin(),
                                                               meanshifted.end(), K);
  ms.iterate(params_.iterations, params_.epsilon);
  FlatKernel Kc(params_.lambda);
  Cluster<vector<Point2f>::iterator, FlatKernel, NormDot> c(ms.begin_out, ms.end_out, Kc);
  vector<int> labels(points.size());
  const int n_Labels = c(labels.begin());
  Mat_<int> ml(labels);

  Mat_<int> idxs;
  sortIdx(ml, idxs, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
  clusters.clear();
  clusters.resize(n_Labels);

  vector<Cloud> clouds(n_Labels);
  vector<Features2d> features2d(n_Labels);

  vector<int> counts(n_Labels);
  for (int i = 0; i < n_Labels; i++)
  {
    counts[i] = countNonZero(Mat(ml == (i + 1)));
    Features2d& f2d = features2d[i];
    features.features().lightCopy(f2d);

    f2d.descriptors = Mat::zeros(counts[i], features.features().descriptors.cols,
                                 features.features().descriptors.type());
    f2d.keypoints.resize(counts[i]);

    Cloud& cloud = clouds[i];
    cloud.resize(counts[i]);

  }
  int prevlabel = -1;
  int offset = 0;
  for (int i = 0; i < idxs.rows; i++, offset++)
  {
    int idx = idxs(i);
    int label = ml(idx) - 1;

    if (label != prevlabel)
    {
      offset = 0;
      prevlabel = label;
    }

    Features2d& f2d = features2d[label];
    Cloud& cloud = clouds[label];
    Mat drow = f2d.descriptors.row(offset);
    drow += features.features().descriptors.row(idx);
    f2d.keypoints[offset] = features.features().keypoints[idx];
    cloud[offset] = features.cloud()[idx];
  }

  for (int i = 0; i < n_Labels; i++)
  {
    clusters[i] = Features3d(features2d[i], clouds[i]);
  }

}

void Clusterer::DrawClusters(const std::vector<Features3d>& f3ds, cv::Mat& image){



  for (size_t i = 0; i < f3ds.size(); i++)
  {
    if(image.empty()){

          f3ds[i].features().image.copyTo(image);
          if(image.channels() != 3){
            cvtColor(image.clone(),image,CV_GRAY2RGB);
          }
        }
    Scalar color((rand() & 125), rand() & 255, rand() & 125);
    color += Scalar::all(125);
    drawKeypoints(image, f3ds[i].features().keypoints, image, color, DrawMatchesFlags::DRAW_OVER_OUTIMG);
  }


}
}
