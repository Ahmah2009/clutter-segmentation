/*
 * Author: Julius Adorf 
 */

// TODO: < > or " " ?
#include <iostream>
#include <limits>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv_candidate/PoseRT.h>
#include <opencv2/highgui/highgui.hpp>
#include <tod/core/Features3d.h>
#include <tod/training/masking.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "tod/training/clouds.h"
#include <cv.h>
#include "misc.h"

// TODO: remove some of these namespaces
using namespace std;
using namespace cv;
using namespace pcl;
using namespace opencv_candidate;

Mat createMask(const PointCloud<PointXYZRGB> & cloud, const PoseRT & pose, const Camera & camera) {
   return tod::cloudMask(cloud, pose, camera); 
}

/** \brief Prepares training data for the feature detection and extraction
 * stage in the training pipeline of ROS package tod_training.
 */
int main(int argc, char *argv[])
{
    // Input:
    // - training base directory
    // - directory with all decompressed training data
    // - name of a specific object

    // TODO: load options from command line using boost::program_options
    // Options opts;
    // options(argc, argv, opts);

    if (argc != 4)
    {
        cerr << "Usage: preparetraining <bags-dir> <base-dir> <object>" << endl;
        return 1;
    }

    // This directory contains all the data from TUM/IAS semantic3d database.
    string bags_dir(argv[1]);
    // This directory hosts the training base that is to be prepared. 
    string train_dir(argv[2]);
    string object(argv[3]);

    cout << bags_dir << endl;
    cout << train_dir << endl;
    cout << object << endl << endl;

    // TODO: platform-independent path construction
    for (int a = -180, i = 0; a <= 180; a += 30, i++) {
        // TODO: namespace
        string pcl_fn = str(boost::format("%s/%s.delimited.pcd/%s_%i_.log.delimited.pcd") % bags_dir % object % object % a);
        string png_fn = str(boost::format("%s/%s/%s_%04i_L.png") % bags_dir % object % object % a);

        cout << "pcl_fn: " << pcl_fn << endl;
        cout << "png_fn: " << png_fn << endl;
        cout << endl;

        // FIXME: get pose from a better estimate
        PoseRT pose;
        pose.rvec = Mat::zeros(3, 1, CV_32F);
        pose.rvec.at<float>(0, 0) = 0;
        pose.rvec.at<float>(1, 0) = 0;
        pose.rvec.at<float>(2, 0) = 0;
        pose.tvec = Mat::zeros(3, 1, CV_32F);

        // FIXME: get camera parameters
        FileStorage fs(str(boost::format("%s/camera.yml") % bags_dir), FileStorage::READ);       
        tod::Camera camera(fs.root());

        // Read full image from semantic 3d training data.
        Mat image = imread(png_fn);
        // Read delimited point cloud from semantic 3d training data.
        PointCloud<PointXYZ> cloudXYZ;
        io::loadPCDFile(pcl_fn, cloudXYZ);
        PointCloud<PointXYZRGB> cloud;
        cloud.points.resize(cloudXYZ.points.size());
        for (unsigned int j = 0; j < cloud.points.size(); j++) {
            cloud.points[j].x = cloudXYZ.points[j].x;
            cloud.points[j].y = cloudXYZ.points[j].y;
            cloud.points[j].z = cloudXYZ.points[j].z;
        }

        
        float ymin = numeric_limits<float>::max();
        float xavg = 0, zavg = 0;
        int n = 0;
        PointCloud<PointXYZRGB>::iterator it = cloud.begin();
        PointCloud<PointXYZRGB>::iterator end = cloud.end();
        while (it != end) {
            xavg += it->x;
            zavg += it->z;
            ymin = min(ymin, it->y);
            n++;
            it++;
        }
        xavg /= n;
        zavg /= n;
        pose.tvec.at<float>(0, 0) = -xavg;
        pose.tvec.at<float>(1, 0) = -ymin;
        pose.tvec.at<float>(2, 0) = -zavg;
        

        // Create mask
        Mat mask = createMask(cloud, pose, camera);

        // Write results to training base
        string b = "image_" + str(boost::format("%04i") % i);
        imwrite(str(boost::format("%s/%s/image_%04i.png") % train_dir % object % i), image);
        imwrite(str(boost::format("%s/%s/image_%04i.png.mask.png") % train_dir % object % i), mask);
        // TODO: make directories if necessary
        FileStorage out(str(boost::format("%s/%s/image_%04i.png.pose.yaml") % train_dir % object % i), FileStorage::WRITE);
        out << PoseRT::YAML_NODE_NAME;
        pose.write(out);
    }

    // list images
    // for v in views do
    //     p = estimate_pose(v)
    //     m = create_mask(v)
    //     p.serialize()
    //     m.serialize()
    // done

    // Output:
    // - black and white mask representing the region of interest for each training image
    // - pose estimation for each training image
}

