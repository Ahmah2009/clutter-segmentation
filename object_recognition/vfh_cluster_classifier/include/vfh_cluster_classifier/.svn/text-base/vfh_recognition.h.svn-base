#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <flann/io/hdf5.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <flann/flann.h>
#include <vector>
#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <vfh_cluster_classifier/common_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "LinearMath/btTransform.h"
#include <terminal_tools/print.h>

namespace bf = boost::filesystem;
namespace ser = ros::serialization;
using namespace terminal_tools;

namespace vfh_classifier
{

  typedef std::pair<int, std::vector<float> > vfh_model_db;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::PointNormal> PointCloudAndNormals;
  typedef std::pair<int, float> index_score;

  inline bool
  ICPredicate (const index_score& d1, const index_score& d2)
  {
    return d1.second < d2.second;
  }

  class VFHRecognizer
  {
  private:
    std::vector<vfh_model_db> models_;
    flann::Matrix<float> data_;
    int knn_;
    flann::Index<flann::ChiSquareDistance<float> >* index_;
    household_objects_database::ObjectsDatabase * database;

    bool
    loadFileList (std::vector<vfh_model_db> &models, const std::string &filename);

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Save the list of file models.
     * \param models the list models
     * \param filename the output file name
     */
    void
    saveFileList (const std::vector<vfh_model_db> &models, const std::string &filename);

    void
    nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > * index, const vfh_model_db &model, int k,
                    flann::Matrix<int> &indices, flann::Matrix<float> &distances);
    void
    computeVFH (pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,
                pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature);

    bool
    getPointCloudFromVFHId (pcl::PointCloud<pcl::PointNormal> & cloud, int vfh_id, boost::shared_ptr<
        household_objects_database::DatabaseView> & view);

  public:
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Initialize the data structures for recognition
     * \param useDB - if true creates the structures from the database
     *              - if false, uses the cached kdtree if found, otherwise builds from DB
     */
    bool
    initialize (bool useDB, bf::path dpath, int linear = 0);
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Given a cluster returns the n-most similar models in models_, their poses
     * and the detection confidence.
     */
    bool
    detect (const sensor_msgs::PointCloud2ConstPtr& msg, int nModels, std::vector<int>& model_ids, std::vector<
        geometry_msgs::Pose> & poses, std::vector<float>& confidences, bool use_fitness_score = true, std::vector<
        boost::shared_ptr<household_objects_database::DatabaseView> > * views = NULL);
  };

  bool
  VFHRecognizer::getPointCloudFromVFHId (pcl::PointCloud<pcl::PointNormal> & cloud, int vfh_id, boost::shared_ptr<
      household_objects_database::DatabaseView> & view)
  {
    if (!database->getViewFromVFHId (vfh_id, view))
      return (false);

    boost::shared_array < uint8_t > bufferRead (new uint8_t[view->view_point_cloud_data_.data ().size ()]);
    memcpy (&bufferRead[0], &(view->view_point_cloud_data_.data ()[0]), view->view_point_cloud_data_.data ().size ());
    ser::IStream streamIn (bufferRead.get (), view->view_point_cloud_data_.data ().size ());
    ser::deserialize (streamIn, cloud);

    return (true);
  }

  void
  VFHRecognizer::computeVFH (pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                             pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,
                             pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature)
  {
    typedef pcl::VFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> VFHEstimator;
    typedef pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> NormalEstimation;
    VFHEstimator vfh;
    NormalEstimation n3d;

    pcl::VoxelGrid < pcl::PointXYZ > grid_;
    pcl::PointCloud < pcl::PointXYZ > grid;
    grid_.setInputCloud (input);
    grid_.setLeafSize (0.01, 0.01, 0.01);
    grid_.filter (grid);

    copyPointCloud (grid, *cloud_normals);

    typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;
    KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > (false);
    normals_tree->setInputCloud (cloud_normals);
    n3d.setRadiusSearch (0.04);
    n3d.setSearchMethod (normals_tree);
    n3d.setInputCloud (cloud_normals);
    n3d.compute (*cloud_normals);

    //check nans...
    int j = 0;
    for (size_t i = 0; i < cloud_normals->points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_normals->points[i].normal_x) || !pcl_isfinite (cloud_normals->points[i].normal_y)
          || !pcl_isfinite (cloud_normals->points[i].normal_z))
        continue;
      cloud_normals->points[j] = cloud_normals->points[i];
      j++;
    }

    cloud_normals->points.resize (j);
    cloud_normals->width = j;
    cloud_normals->height = 1;

    pcl::KdTree<pcl::PointNormal>::Ptr vfh_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > ();
    vfh.setSearchMethod (vfh_tree);
    vfh.setInputCloud (cloud_normals);
    vfh.setInputNormals (cloud_normals);
    vfh.compute (*vfh_signature);
  }

  bool
  VFHRecognizer::loadFileList (std::vector<vfh_model_db> &models, const std::string &filename)
  {
    std::ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
      return (false);

    std::string line;
    while (!fs.eof ())
    {
      getline (fs, line);
      if (line.empty ())
        continue;
      vfh_model_db m;
      m.first = atoi (line.c_str ());
      models.push_back (m);
    }
    fs.close ();
    return (true);
  }

  void
  VFHRecognizer::saveFileList (const std::vector<vfh_model_db> &models, const std::string &filename)
  {
    std::ofstream fs;
    fs.open (filename.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
      fs << models[i].first << "\n";
    fs.close ();
  }

  void
  VFHRecognizer::nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > * index, const vfh_model_db &model,
                                 int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
  {
    flann::Matrix<float> p = flann::Matrix<float> (new float[model.second.size ()], 1, model.second.size ());
    memcpy (&p.data[0], &model.second[0], p.cols * p.rows * sizeof(float));

    indices = flann::Matrix<int> (new int[k], 1, k);
    distances = flann::Matrix<float> (new float[k], 1, k);
    index->knnSearch (p, indices, distances, k, flann::SearchParams (512));
    p.free ();
  }

  bool
  VFHRecognizer::initialize (bool useDB, bf::path dpath, int linear)
  {
    bf::path training_data_h5_file_name = dpath / "training_data.h5";
    bf::path training_data_list_file_name = dpath / "training_data.list";
    bf::path index_filename = dpath / "kdtree.idx";

    if (useDB || !boost::filesystem::exists (training_data_h5_file_name)
        || !boost::filesystem::exists (training_data_list_file_name) || !boost::filesystem::exists (index_filename))
    {
      //read training data from DB and build kdtree
      vfh_cluster_classifier::buildTreeFromDB (models_, data_);
      if (linear == 0)
        index_ = new flann::Index<flann::ChiSquareDistance<float> > (data_, flann::LinearIndexParams ());
      else
        index_ = new flann::Index<flann::ChiSquareDistance<float> > (data_, flann::KDTreeIndexParams (4));
      index_->buildIndex ();

      if (useDB)
      {
        //files could exists, try to delete them
        if (boost::filesystem::exists (training_data_h5_file_name))
          boost::filesystem::remove (training_data_h5_file_name);

        if (boost::filesystem::exists (training_data_list_file_name))
          boost::filesystem::remove (training_data_list_file_name);

        if (boost::filesystem::exists (index_filename))
          boost::filesystem::remove (index_filename);
      }

      if (!boost::filesystem::exists (dpath))
      {
        ROS_WARN ("%s does not exist. Not able to save training_data!", dpath.string ().c_str ());
      }
      else
      {
        //SAVE the kdtree + training_list to the database
        flann::save_to_file (data_, training_data_h5_file_name.string ().c_str (), "training_data");
        saveFileList (models_, training_data_list_file_name.string ().c_str ());
        index_->save (index_filename.string ().c_str ());
      }
    }
    else
    {
      loadFileList (models_, training_data_list_file_name.string ());
      flann::load_from_file (data_, training_data_h5_file_name.string (), "training_data");
      print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data_.rows,
                       training_data_h5_file_name.string ().c_str (), training_data_list_file_name.string ().c_str ());

      index_
          = new flann::Index<flann::ChiSquareDistance<float> > (
                                                                data_,
                                                                flann::SavedIndexParams (
                                                                                         index_filename.string ().c_str ()));
      index_->buildIndex ();
    }

    //Connect to database
    database = new household_objects_database::ObjectsDatabase ("wgs36.willowgarage.com", "5432", "willow", "willow",
                                                                "household_objects");

    return true;
  }

  bool
  VFHRecognizer::detect (const sensor_msgs::PointCloud2ConstPtr& msg, int nModels, std::vector<int>& model_ids,
                         std::vector<geometry_msgs::Pose> & poses, std::vector<float>& confidences,
                         bool use_fitness_score, std::vector<
                             boost::shared_ptr<household_objects_database::DatabaseView> >* views)
  {
    PointCloud::Ptr cloud (new PointCloud ());
    PointCloudAndNormals::Ptr cloud_normals (new PointCloudAndNormals ());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_signature (new pcl::PointCloud<pcl::VFHSignature308> ());

    pcl::fromROSMsg (*msg, *cloud);
    computeVFH (cloud, cloud_normals, vfh_signature);

    assert (vfh_signature->points.size () == 1);
    float* hist = vfh_signature->points[0].histogram;
    std::vector<float> std_hist (hist, hist + 308);
    vfh_model_db histogram (-1, std_hist);

    size_t k = 31;
    flann::Matrix<int> indices;
    flann::Matrix<float> distances;

    nearestKSearch (index_, histogram, k, indices, distances);

    Eigen3::Vector4f centroid;
    compute3DCentroid (*cloud_normals, centroid);
    pcl::PointCloud < pcl::PointNormal > cloud_xyz_demean_input;
    pcl::demeanPointCloud (*cloud_normals, centroid, cloud_xyz_demean_input);

    //Perform fitness score comparison over the k models returned by VFH
    //We can activate or turn off this part

    double fscore;
    std::vector<index_score> index_scores;

    for (size_t i = 0; i < k; ++i)
    {
      int vfh_id = models_.at (indices[0][i]).first;

      if (use_fitness_score)
      {
        pcl::PointCloud < pcl::PointNormal > cloud_xyz;
        boost::shared_ptr < household_objects_database::DatabaseView > view;
        getPointCloudFromVFHId (cloud_xyz, vfh_id, view);
        if (cloud_xyz.points.size () == 0)
          break;

        //Demean the cloud
        Eigen3::Vector4f centroid;
        compute3DCentroid (cloud_xyz, centroid);
        pcl::PointCloud < pcl::PointNormal > cloud_xyz_demean;
        pcl::demeanPointCloud (cloud_xyz, centroid, cloud_xyz_demean);

        //Compute fitness score
        pcl::IterativeClosestPoint < pcl::PointNormal, pcl::PointNormal > reg;
        reg.setInputCloud (cloud_xyz_demean_input.makeShared ());
        reg.setInputTarget (cloud_xyz_demean.makeShared ());
        fscore = reg.getFitnessScore ();
      }
      else
      {
        fscore = distances[0][i];
      }

      index_score is;
      is.first = i;
      is.second = fscore;
      index_scores.push_back (is);
    }

    if (use_fitness_score)
    {
      //Sort index_scores by fitness score
      std::sort (index_scores.begin (), index_scores.end (), ICPredicate);
    }

    //Populate output structures
    for (size_t i = 0; i < (size_t)nModels; ++i)
    {
      int vfh_id = models_.at (indices[0][index_scores.at (i).first]).first;
      boost::shared_ptr < household_objects_database::DatabaseView > view;
      database->getViewFromVFHId (vfh_id, view);
      poses.push_back (view->view_transform_.data ().pose_);
      model_ids.push_back (view->scaled_model_id_.data ());
      confidences.push_back (index_scores.at (i).second);

      //(optionally populate views...)
      if (views != NULL)
      {
        views->push_back (view);
      }
    }
  }
}
