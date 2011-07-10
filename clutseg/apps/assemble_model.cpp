/**
 * Author: Julius Adorf
 */

#include "clutseg/viz.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <boost/program_options.hpp>
    #include <cv.h>
    #include <iostream>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl_ros/transforms.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/filters/radius_outlier_removal.h>
    #include <tod/core/TexturedObject.h>
    #include <tod/core/TrainingBase.h>
    #include <tod/detecting/Loader.h>
    #include <tod/detecting/Tools.h>
    // Put this include at the very end, otherwise the compiler will
    // complain with lengthy messsages
    #include <opencv2/core/eigen.hpp>
    #include <opencv2/highgui/highgui.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace pcl;
using namespace std;
namespace po = boost::program_options;

struct Opts {
    string modelbase_dir;
    string object;
    string pcd_out_file;
};

int options(int argc, char ** argv, Opts& opts) {
    string modelbase = "modelbase";
    string object = "object";
    string pcd_out = "pcd_out";

    po::options_description h("Hidden options");
    h.add_options()
        (modelbase.c_str(), po::value<string>(&opts.modelbase_dir), "modelbase directory")
        (object.c_str(), po::value<string>(&opts.object), "assemble model for this object")
        (pcd_out.c_str(), po::value<string>(&opts.pcd_out_file), "destination point-cloud file");

    po::options_description d("Allowed options");
    d.add_options()("help", "Print this help message.");

    po::positional_options_description p;
    p.add(modelbase.c_str(), 1);
    p.add(object.c_str(), 1);
    p.add(pcd_out.c_str(), 1);

    po::options_description c;
    c.add(d).add(h);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(c).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help") || vm.count(modelbase) == 0 || vm.count(object) == 0 || vm.count(pcd_out) == 0) {
        cout << "usage: \033\[1massemble_model\033[0m [options] " + modelbase + " " + object + " " + pcd_out << endl;
        cerr << endl <<
            "Reads in a model from the modelbase (a.k.a. TrainingBase) and generates a" << endl <<
            "point cloud file that contains all model points (corresponding to features)" << endl <<
            "described in its object coordinate system." << endl;
        cout << endl;
        cout << d << endl;
        return 1;
    }

    return 0;
}

int main(int argc, char **argv) {
    Opts opts;
    if (options(argc, argv, opts)) {
        return 1;
    }

    // Loading all of them is a little bit of overkill
    vector<Ptr<tod::TexturedObject> > objects;
    tod::Loader loader(opts.modelbase_dir);
    loader.readTexturedObjects(objects);

    int i = -1;
    Ptr<tod::TexturedObject> object = NULL;
    BOOST_FOREACH(const Ptr<tod::TexturedObject> & obj, objects) {
        i++;
        if (obj->name == opts.object) {
            object = obj;
        }
    }

    if (i == -1) {
        cerr << "Object " << opts.object << " does not exist." << endl;
        cerr << "Objects in modelbase: ";
        BOOST_FOREACH(const Ptr<tod::TexturedObject> & obj, objects) {
            cerr << obj->name << " ";
        }
        cerr << endl;
        return -1;
    }

    PointCloud<PointXYZ> model;
    BOOST_FOREACH(const tod::Features3d & f3d, object->observations) {
        // Annoying, the model features are stored in view coordinates need to
        // retrieve the object-view transformation for each view.  The
        // object-view transformation is stored as the extrinsic parameters of
        // the camera. We need to invert it to get hold of the sought-after
        // view-object transformation that brings points in view coordinates
        // into the object coordinate system. <R, t> pose is the object-view
        // transformation.
        PoseRT pose = f3d.camera().pose;

        vector<Point3d> op;
        BOOST_FOREACH(const Point3f pf, f3d.cloud()) {
            Point3d pd;
            pd.x = (double) pf.x;
            pd.y = (double) pf.y;
            pd.z = (double) pf.z;
            op.push_back(pd);
        }
        Mat mop(op);
        cv::Mat R;
        PoseRT inverted = tod::Tools::invert(pose);
        cv::Rodrigues(inverted.rvec, R);
        cv::transform(mop, mop, R);
        BOOST_FOREACH(Point3d & pd, op) {
            pd += inverted.tvec.at<cv::Point3d>();
        }

        PointCloud<PointXYZ> view_cloud;
        cout << f3d.cloud().size() << endl;
        cout << f3d.features().keypoints.size() << endl;
        BOOST_FOREACH(const Point3d pd, op) {
            PointXYZ p;
            p.x = pd.x;
            p.y = pd.y;
            p.z = pd.z;
            view_cloud.push_back(p);
        }

        model += view_cloud;

        /* This is a second way to do the transform, leads to the same results
        vector<int> indices;
        pcl::removeNaNFromPointCloud(view_cloud, view_cloud, indices);

        Eigen::Matrix<float, 3, 3> R;
        Eigen::Vector3f t;
        cv::cv2eigen(pose.tvec, t);
        cv::Mat cvR;
        cv::Rodrigues(pose.rvec, cvR);
        cv::cv2eigen(cvR, R);

        Eigen::Affine3f translate(Eigen::Translation3f(-t));
        pcl::transformPointCloud(view_cloud, view_cloud, translate);

        Eigen::Affine3f rotate(Eigen::AngleAxisf(R.transpose()));
        pcl::transformPointCloud(view_cloud, view_cloud, rotate);

        model += view_cloud; */
    }

    io::savePCDFileASCII(opts.pcd_out_file, model);
    
    return 0;
}
