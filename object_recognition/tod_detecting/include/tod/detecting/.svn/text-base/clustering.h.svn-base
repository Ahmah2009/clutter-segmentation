/*
 * clustering.h
 *
 *  Created on: Nov 4, 2010
 *      Author: ethan
 */

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "tod/core/Features3d.h"

#include <vector>

namespace tod
{

typedef std::vector<Features3d> Clusters;

/** \brief Interface for clustering algorithms
 *  \ingroup clustering
 */
class Clusterer
{
public:

  virtual ~Clusterer()
  {
  }
  /** \brief Break a Features3d object into multiple clusters to search for, e.g. use Mean Shift Clustering.
   *  \param features The features to break into clusters.
   *                  Depending on the algorithm this may be done in descriptor space or image space, 3d space, etc..
   *  \param [out] clusters The resulting clusters.
   */
  virtual void cluster(const Features3d& features, Clusters& clusters) = 0;
  static void DrawClusters(const Clusters& f3ds, cv::Mat& image);
};

/** \brief Parameters for the meanshift algorithm
 * \ingroup clustering
 */
struct MeanShiftParams : public Serializable
{
  MeanShiftParams(int iterations = 100, double lambda = 50, double epsilon = 1, bool use3d = false) :
    iterations(iterations), lambda(lambda), epsilon(epsilon), use3d(use3d)
  {

  }
  virtual void write(cv::FileStorage& fs) const;
  virtual void read(const cv::FileNode& fn);
  int iterations;
  double lambda; //!< this is the radius of the the clustering ball or window
  double epsilon; //!< this is the point at which to considered the clustering algorithm finished.
  bool use3d;

};

/** \brief Class encompassing the Mean Shift Clustering algorithm, that helps break apart an image of
 *  a cluttered scene so that descriptor matching converges more robustly.
 *
 *  Can take advantage of 3d information for clustering, or solely 2d...
 *
 *  \ingroup clustering
 *
 */
class MeanShiftClustering : public Clusterer
{
public:
  MeanShiftClustering(const MeanShiftParams& params);
  virtual void cluster(const Features3d& features, Clusters& clusters);
private:
  MeanShiftParams params_;
};
}/*namespace tod*/
#endif /* CLUSTERING_H_ */
