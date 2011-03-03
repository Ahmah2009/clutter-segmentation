#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/convex_hull.h"

#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/registration/registration.h"

#include "tod/core/PoseRT.h"
#include "tod/core/Camera.h"
#include "tod/training/clouds.h"
#include <tod/core/Features3d.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <tod/training/masking.h>

#include <pcl/filters/radius_outlier_removal.h>

namespace tod{


typedef tod::PointCloudMasker::cloud_t cloud_t;
typedef cloud_t::PointType point_t;
cv::Mat cloudMask(const cloud_t& cloud, const PoseRT& pose, const Camera& camera)
  {

    cloud_t::Ptr cloud_filtered(new cloud_t);
    cloud_t::Ptr cloud_projected(new cloud_t);

    Eigen::Matrix<float, 3, 3> R;
    Eigen::Vector3f T;
    cv::cv2eigen(pose.tvec, T);
    cv::Mat cvR;
    cv::Rodrigues(pose.rvec,cvR);
    cv::cv2eigen(cvR, R);

    Eigen::Affine3f transform;
    transform = Eigen::AngleAxisf(R.transpose());
    transform *= Eigen::Translation3f(-T);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(cloud, *cloud_filtered, indices);
    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform);

    //use the pass through filter to give us only a small box of points around our fudicial marker
    pcl::PassThrough<point_t> pass;
    pass.setInputCloud(cloud_filtered);
    float box = 0.25;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.01, box);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(- box, box);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(- box, + box);
    pass.filter(*cloud_filtered);

    pcl::RadiusOutlierRemoval<cloud_t::PointType> ror;
    ror.setRadiusSearch(0.01); //2 centimeter search radius
    ror.setMinNeighborsInRadius(50);
    ror.setInputCloud(cloud_filtered);
    ror.filter(*cloud_filtered);

    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform.inverse());

    if (cloud_filtered->points.size() == 0)
      return cv::Mat();

    std::vector<cv::Point3f> points;
    tod::PCLToPoints(points, *cloud_filtered);

    cv::Mat mask = cv::Mat::zeros(camera.image_size, CV_8UC1);

    tod::drawProjectedPoints(camera, points, mask,2,3);

    return mask;

  }
}
