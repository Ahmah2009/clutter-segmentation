/**
 * Author: Julius Adorf
 */
#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/foreach.hpp>
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
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace pcl;
using namespace std;
using namespace tod;

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << "Usage: assemble_model <base> <object> <dst>" << endl;
        cerr << endl <<
            "Reads in a model from the modelbase (a.k.a. TrainingBase) and generates a\n"
            "point cloud file that contains all model points (corresponding to features)\n"
            "described in its object coordinate system." << endl;
        return -1;
    }

    // Loading all of them is a little bit of overkill
    vector<Ptr<TexturedObject> > objects;
    Loader loader(argv[1]);
    loader.readTexturedObjects(objects);

    int i = -1;
    Ptr<TexturedObject> object;
    BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects) {
        i++;
        if (obj->name == argv[2]) {
            object = obj;
        }
    }

    if (i == -1) {
        cerr << "Object " << argv[2] << " does not exist." << endl;
        cerr << "Objects in modelbase: ";
        BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects) {
            cerr << obj->name << " ";
        }
        cerr << endl;
        return -1;
    }

    PointCloud<PointXYZ> model;
    BOOST_FOREACH(const Features3d & f3d, object->observations) {
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
        PoseRT inverted = Tools::invert(pose);
        cv::Rodrigues(inverted.rvec, R);
        cv::transform(mop, mop, R);
        BOOST_FOREACH(Point3d & pd, op) {
            pd += inverted.tvec.at<cv::Point3d>();
        }

        PointCloud<PointXYZ> view_cloud;
        cout << f3d.cloud().size() << endl;
        cout << f3d.features().keypoints.size() << endl;
        // What the hell, this this a vector
        BOOST_FOREACH(const Point3d pd, op) {
            PointXYZ p;
            p.x = pd.x;
            p.y = pd.y;
            p.z = pd.z;
            view_cloud.push_back(p);
        }

        model += view_cloud;
    /*
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

    io::savePCDFileASCII(argv[3], model);

    return 0;
}
