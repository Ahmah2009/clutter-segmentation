/*
 * Author: Julius Adorf 
 */

// TODO: < > or " " ?
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv_candidate/PoseRT.h>
#include <opencv2/highgui/highgui.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <cv.h>
#include "misc.h"

// TODO: remove some of these namespaces
using namespace std;
using namespace cv;
using namespace pcl;
using namespace opencv_candidate;

void createMask(const PointCloud<PointXYZ> & cloud, const Mat & image, const Mat & mask) {

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
        string pcl_fn = bags_dir + "/" + object + ".delimited.pcd/" +
            object + "_" + boost::lexical_cast<string>(a) + "_.log.delimited.pcd";
        string png_fn = bags_dir + "/" + object + "/" +
            object + "_" + str(boost::format("%04i") % a) + "_L.png";
        cout << "pcl_fn: " << pcl_fn << endl;
        cout << "png_fn: " << png_fn << endl;
        cout << endl;
        // FIXME: get pose from a better estimate
        PoseRT p;
        // Read full image from semantic 3d training data.
        Mat image = imread(png_fn);
        // Read delimited point cloud from semantic 3d training data.
        PointCloud<PointXYZ> cloud;
        io::loadPCDFile(pcl_fn, cloud);
        // Create mask
        Mat mask = Mat::zeros(image.size(), CV_8U);
        createMask(cloud, image, mask);
        // Write results to training base
        string b = "image_" + str(boost::format("%04i") % i);
        imwrite(train_dir + "/" + object + "/" + b + ".png", image);
        imwrite(train_dir + "/" + object + "/" + b + ".png.mask.png", mask);

        // TODO: make directories if necessary
        FileStorage out(train_dir + "/" + object + "/" + b + ".png.pose.yaml", FileStorage::WRITE);
        out << PoseRT::YAML_NODE_NAME;
        p.write(out);
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

